"""
Gymnasium environment for GL-FOPID parameter tuning via Gazebo simulation.

ARCHITECTURE: the ROS 2/Gazebo-specific mechanics (setting parameters on
the live row_nav_node, resetting the robot's pose in Gazebo, collecting
/row_heading_error + /row_lateral_offset + IMU angular velocity over an
episode) are isolated behind a small Backend interface, injected into
GLFOPIDGazeboEnv rather than hardcoded into it. This is not over-
engineering for its own sake: it is what makes it possible to verify this
environment's structural correctness (Gymnasium API compliance, the
reward/observation wiring) with MockBackend in a sandbox that has no
rclpy, no Gazebo, and no running simulation -- and it is exactly what was
verified before this was trusted, not assumed. RealRosGazeboBackend below
is the genuine ROS 2/Gazebo implementation; it CANNOT be exercised in
that same sandbox and needs real-environment verification on your actual
Orin with Gazebo running -- see docs/GL_FOPID_RL_TUNING_GUIDE.md for
the exact verification steps to run before trusting a training session's
results.

EPISODE = ONE fixed GL-FOPID parameter set, ONE fixed-duration simulated
drive, then terminate -- see reward.py's own module docstring for why
this single-step-per-episode design matches the actual stated objective
("export one final static parameter set") rather than a more complex
multi-step formulation.
"""
import time
from abc import ABC, abstractmethod

import numpy as np
import gymnasium as gym
from gymnasium import spaces

from reward import (
    action_to_params, build_observation, compute_reward,
    ACTION_DIM, OBS_DIM,
)


class EpisodeBackend(ABC):
    """What GLFOPIDGazeboEnv needs from "the simulation", independent of
    how that's actually provided. Implement this once for real ROS 2/
    Gazebo (RealRosGazeboBackend below), and once for testing (MockBackend
    below) -- the Env class itself never needs to know which one it has."""

    @abstractmethod
    def run_episode(self, params: dict, duration_s: float,
                     abort_lateral_m: float, abort_heading_rad: float):
        """Apply `params` to the live controller, reset the robot to the
        row start, run for up to `duration_s` seconds of sim time
        (ending early if the robot exceeds abort_lateral_m or
        abort_heading_rad -- both a real training-time optimization,
        since real-time-bound Gazebo episodes are wall-clock expensive,
        and a real safety property: a badly-tuned parameter set
        shouldn't be left running against a real chassis, sim or not, any
        longer than needed to confirm it's bad).

        Returns: (heading_rms, lateral_rms, vibration_rms, aborted: bool)
        """
        raise NotImplementedError


class MockBackend(EpisodeBackend):
    """Deterministic, ROS/Gazebo-free stand-in used ONLY to verify
    GLFOPIDGazeboEnv's own structural correctness (Gymnasium API
    compliance, reward/observation wiring) -- NOT a substitute for real
    validation against actual simulated physics. Returns metrics as a
    simple, known function of the requested parameters specifically so a
    test can assert the environment correctly threads a given action
    through to a given reward, which is what actually matters to verify
    here -- not realistic control-system behaviour, which this mock does
    not and cannot provide."""

    def __init__(self, seed=0):
        self.rng = np.random.default_rng(seed)

    def run_episode(self, params, duration_s, abort_lateral_m, abort_heading_rad):
        # Deliberately simple, deterministic-plus-small-noise function of
        # the parameters, chosen only so "better" parameters (closer to
        # this workspace's shipped defaults) score better -- gives a real
        # gradient for a quick smoke-test training run to visibly follow,
        # without claiming any resemblance to the real chassis's dynamics.
        target = {'Kp_theta': 1.2, 'Ki_theta': 0.25, 'Kd_theta': 0.30,
                  'fopid_lambda': 0.4, 'fopid_mu': 0.7, 'k_cross_track': 0.8}
        err = sum((params[k] - target[k]) ** 2 for k in target)
        noise = self.rng.normal(0, 0.01)
        heading_rms = max(0.0, 0.05 + 0.3 * err + noise)
        lateral_rms = max(0.0, 0.02 + 0.15 * err + noise)
        vibration_rms = max(0.0, 0.01 + 0.1 * err + noise)
        aborted = heading_rms > abort_heading_rad or lateral_rms > abort_lateral_m
        return heading_rms, lateral_rms, vibration_rms, aborted


class RealRosGazeboBackend(EpisodeBackend):
    """
    The genuine ROS 2 + Gazebo implementation. Requires rclpy and a
    running gazebo_sim.launch.py (see docs/GL_FOPID_RL_TUNING_GUIDE.md).
    All ROS-specific imports are deferred into __init__ and the methods
    that need them (standard Python pattern for an optional dependency --
    NOT scattered inline __import__() calls, which an earlier draft of
    this file used and which made the actual service-call logic hard to
    read and review; fixed before this was trusted) so this whole module
    stays importable, and MockBackend usable, on a machine without rclpy
    installed at all.

    NOT VERIFIED BY EXECUTION -- this sandbox has neither rclpy nor a
    running Gazebo instance. Written using standard, well-established
    ROS 2/Gazebo Classic APIs (rclpy's parameter-setting client,
    gazebo_msgs/SetEntityState) rather than anything obscure, but you
    should still run the smoke test in the guide (a handful of episodes,
    watching Gazebo directly) before trusting a long training run's
    results.
    """

    def __init__(self, robot_entity_name='strawberry_amr',
                 row_nav_node_name='row_nav_node',
                 start_pose=(-0.5, 0.0, 0.0)):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32, Bool
        from sensor_msgs.msg import Imu
        from gazebo_msgs.srv import SetEntityState

        self.robot_entity_name = robot_entity_name
        self.start_pose = start_pose
        self._Bool = Bool

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self.node = Node('gl_fopid_rl_trainer')

        self._heading_buf = []
        self._lateral_buf = []
        self._vibration_buf = []
        self.node.create_subscription(Float32, '/row_heading_error',
                                      lambda m: self._heading_buf.append(m.data), 10)
        self.node.create_subscription(Float32, '/row_lateral_offset',
                                      lambda m: self._lateral_buf.append(m.data), 10)
        self.node.create_subscription(
            Imu, '/imu/data',
            lambda m: self._vibration_buf.append(
                (m.angular_velocity.x ** 2 + m.angular_velocity.y ** 2
                 + m.angular_velocity.z ** 2) ** 0.5), 10)

        self.auto_pub = self.node.create_publisher(Bool, '/autonomy_enable', 10)
        self.mission_pub = self.node.create_publisher(Bool, '/mission_active', 10)
        self.set_state_client = self.node.create_client(
            SetEntityState, '/gazebo/set_entity_state')
        self.param_client = self.node.create_client(
            self._set_parameters_srv_type(), f'/{row_nav_node_name}/set_parameters')

    @staticmethod
    def _set_parameters_srv_type():
        from rcl_interfaces.srv import SetParameters
        return SetParameters

    def _apply_params(self, params: dict):
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

        req = SetParameters.Request(parameters=[
            Parameter(name=name,
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                           double_value=float(value)))
            for name, value in params.items()
        ])
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                'row_nav_node /set_parameters service unavailable -- is '
                'gazebo_sim.launch.py actually running?')
        future = self.param_client.call_async(req)
        self._rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

    def _reset_robot_pose(self):
        from gazebo_msgs.srv import SetEntityState
        from gazebo_msgs.msg import EntityState

        x, y, yaw = self.start_pose
        state = EntityState()
        state.name = self.robot_entity_name
        state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, 0.05
        state.pose.orientation.z = float(np.sin(yaw / 2.0))
        state.pose.orientation.w = float(np.cos(yaw / 2.0))
        req = SetEntityState.Request(state=state)
        if not self.set_state_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                '/gazebo/set_entity_state unavailable -- is Gazebo actually running?')
        future = self.set_state_client.call_async(req)
        self._rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

    def run_episode(self, params, duration_s, abort_lateral_m, abort_heading_rad):
        self._apply_params(params)
        self._reset_robot_pose()
        self._heading_buf.clear()
        self._lateral_buf.clear()
        self._vibration_buf.clear()

        self.auto_pub.publish(self._Bool(data=True))
        self.mission_pub.publish(self._Bool(data=True))

        # KNOWN LIMITATION, flagged rather than silently patched: this is
        # WALL-CLOCK time, not simulated Gazebo time. If Gazebo's
        # real_time_factor drops below 1.0 (plausible under load, e.g. on a
        # Jetson), this loop still exits at wall-clock duration_s, meaning
        # LESS simulated driving time than episode_duration_s actually
        # configures -- silently under-testing each parameter set with no
        # warning. The correct fix is self.node.get_clock().now() with
        # use_sim_time=True on this node PLUS Gazebo's /clock actually being
        # published and consumed project-wide -- checked, and use_sim_time
        # is not currently set anywhere in this workspace for any node
        # (grep confirms exactly one hit, hardcoded False, in
        # robot_description/launch/description.launch.py). Making this one
        # node sim-time-aware in isolation, without verifying /clock is
        # actually being published, risks a WORSE failure than the current
        # one: get_clock().now() would simply stop advancing if /clock
        # never arrives, hanging every episode instead of merely
        # under-timing it. Left as wall-clock, documented, rather than
        # applying an untestable fix (no live Gazebo in this dev
        # environment) that could silently make things worse.
        t0 = time.monotonic()
        aborted = False
        while time.monotonic() - t0 < duration_s:
            self._rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._lateral_buf and abs(self._lateral_buf[-1]) > abort_lateral_m:
                aborted = True
                break
            if self._heading_buf and abs(self._heading_buf[-1]) > abort_heading_rad:
                aborted = True
                break

        self.auto_pub.publish(self._Bool(data=False))

        def rms(buf):
            return float(np.sqrt(np.mean(np.square(buf)))) if buf else 0.0

        return rms(self._heading_buf), rms(self._lateral_buf), rms(self._vibration_buf), aborted


class GLFOPIDGazeboEnv(gym.Env):
    """
    Gymnasium environment: one episode = one GL-FOPID parameter set,
    tested for `episode_duration_s` seconds of simulated driving, then
    terminated. See module docstring and reward.py for the full design
    reasoning.
    """
    metadata = {'render_modes': []}

    def __init__(self, backend: EpisodeBackend, episode_duration_s=25.0,
                 abort_lateral_m=0.35, abort_heading_rad=1.2,
                 reward_weights=None, reward_scale=1.0,
                 instability_penalty=5.0):
        super().__init__()
        self.backend = backend
        self.episode_duration_s = episode_duration_s
        self.abort_lateral_m = abort_lateral_m
        self.abort_heading_rad = abort_heading_rad
        self.reward_weights = reward_weights or {}
        self.reward_scale = reward_scale
        self.instability_penalty = instability_penalty

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(ACTION_DIM,),
                                       dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(OBS_DIM,),
                                            dtype=np.float32)

        self._prev_reward = 0.0
        self._prev_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self._best_reward = -np.inf
        self._best_params = None

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        obs = build_observation(self._prev_reward, self._prev_action, self.reward_scale)
        return obs, {}

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        params = action_to_params(action)

        heading_rms, lateral_rms, vibration_rms, aborted = self.backend.run_episode(
            params, self.episode_duration_s, self.abort_lateral_m, self.abort_heading_rad)

        penalty = self.instability_penalty if aborted else 0.0
        reward = compute_reward(heading_rms, lateral_rms, vibration_rms,
                                instability_penalty=penalty, **self.reward_weights)

        if reward > self._best_reward:
            self._best_reward = reward
            self._best_params = params

        self._prev_reward = reward
        self._prev_action = action

        obs = build_observation(reward, action, self.reward_scale)
        info = {
            'params': params, 'heading_rms': heading_rms, 'lateral_rms': lateral_rms,
            'vibration_rms': vibration_rms, 'aborted': aborted,
            'best_reward_so_far': self._best_reward, 'best_params_so_far': self._best_params,
        }
        # terminated=True: every episode is exactly one step, by design --
        # see module docstring
        return obs, reward, True, False, info
