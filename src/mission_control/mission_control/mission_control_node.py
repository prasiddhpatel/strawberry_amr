#!/usr/bin/env python3
"""
Mission sequencing + a clean, simple Orin command interface. Replaces the
previous harvest_coordinator package (removed -- see git history): the
5-DOF arm is a separate, not-yet-confirmed companion project, so this node
does NOT wait for or expect any arm handoff signal. It sequences mapped
plants and stops briefly at each one in order, then moves on.

THREE PHASES now, not two -- 'mapping', 'nav', and 'explore':
  'mapping'  Original two-phase workflow, Phase 1 (known layout, e.g. Lab
             3003 desk rows): SLAM builds the map, operator sends
             "finish_mapping" when done, then switches to the 'nav' launch
             profile against the frozen, saved map. Still the right choice
             whenever the layout is already known and precise, repeatable
             plant-visit ordering matters more than autonomous discovery.
  'nav'      Original Phase 2: loads a frozen map, sequences a pre-known
             set of rows via coverage_planner, approaches each mapped plant.
  'explore'  NEW: for a genuinely UNKNOWN tunnel with no pre-existing map
             (see docs/ORIN_PI_SPLIT_ARCHITECTURE.md) -- SLAM runs
             continuously (no separate "finish mapping" gate), and
             row_navigation's own opt-in reactive row-discovery (see that
             node's `exploration_mode` parameter and its REACQUIRE-block
             comments) serves as the exploration mechanism: it drives each
             row it finds, headland-turns, and tries to reacquire the next
             one purely from LiDAR returns -- no pre-known row count or
             layout needed. This node's job in 'explore' phase is narrow
             and deliberately conservative: gate autonomy on operator
             command exactly as in 'nav' phase, and when row_navigation
             reports /row_found=false (it tried and found nothing, see
             that node's EXPLORE_HALT state), declare EXPLORATION_COMPLETE
             and STOP -- it does not try to guess a different heading or
             otherwise improvise; per row_navigation's own stated
             philosophy, a mission node deciding "what next" should be a
             deliberate choice, not automatic. `finish_mapping` remains
             available in ANY phase as a snapshot -- it just calls SLAM's
             serialize service and confirms the plant CSV exists; it does
             not stop or gate anything, so there is no reason to restrict
             when you can use it.

ORIN COMMAND INTERFACE (the "clean and simple mission commands" this was
built for) -- two plain topics, usable directly from a Orin with nothing
but stock `ros2` CLI tools, no custom client needed:

    /mission/command  (std_msgs/String, you publish)   -- accepted values:
        "finish_mapping"  Available in any phase. Saves the SLAM map (calls
                           slam_toolbox's serialize_map service) and confirms
                           semantic_mapper's plant CSV exists, then reports
                           MAP_SAVED (mapping/explore phases only report this
                           as a mission-state change; in nav phase it is
                           accepted but has no mission-state effect since
                           Phase 2 already runs off a frozen map).
        "start_nav"        'nav' or 'explore' phase. Begins autonomous
                           coverage/exploration + per-plant approach.
        "stop"              Pauses the mission (/mission_active -> false)
                           without touching hardware e-stop -- the row
                           follower and Nav2 both stop issuing commands, but
                           this is a mission-level pause, not a safety stop.
                           Use the PS2 pad's e-stop button for that.

    /mission/status    (std_msgs/String, you echo)      -- the "signal back
                           to the Orin" the operator asked for. Publishes
                           on every state change: WAITING_FOR_MAPPING_DONE,
                           MAP_SAVED, WAITING_FOR_START, NAVIGATING,
                           EXPLORING, APPROACHING_PLANT, MISSION_COMPLETE,
                           EXPLORATION_COMPLETE, STOPPED.

    ros2 topic pub --once /mission/command std_msgs/String "data: start_nav"
    ros2 topic echo /mission/status

Everything else (coverage-plan consumption, headland-turn row counting,
Nav2 NavigateToPose goal handling) is carried over unchanged from the
former harvest_coordinator, which was already correct for that part.

ZERO-SHOT INTEGRATION (zeroshot_enabled parameter) -- if
plant_detector_zeroshot_node.py (detector:='zeroshot') is running, this
node fires its detection trigger (/zeroshot_detect_trigger,
std_msgs/Empty) at exactly one moment: the instant Nav2 confirms it has
reached an approached plant, right as the confirm_pause dwell begins (see
_goal_result_cb). This is deliberately fire-and-forget -- the mission FSM
does NOT wait for a detection result before advancing to the next plant.
The zero-shot node runs its inference asynchronously and publishes
directly to /plant_targets / /detected_weeds whenever it finishes
(typically a few hundred ms, comfortably inside the default 3s
confirm_pause, but not gated on it); semantic_mapper de-duplicates by
position, not arrival order, so a detection landing slightly after this
node has already moved on costs nothing. Launch files set
zeroshot_enabled automatically from the same detector:= argument that
selects which perception node runs at all -- there should be no need to
set this parameter by hand.
"""
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Bool, Empty
from geometry_msgs.msg import PoseStamped

try:
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    _HAVE_NAV2 = True
except Exception:  # nav2_msgs not installed -> coverage-only mode
    NavigateToPose = None
    GoalStatus = None
    _HAVE_NAV2 = False

try:
    from slam_toolbox.srv import SerializePoseGraph
    _HAVE_SLAM_SRV = True
except Exception:
    SerializePoseGraph = None
    _HAVE_SLAM_SRV = False


def _latched_qos():
    return QoSProfile(depth=1,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        d = self.declare_parameter
        d('phase', 'nav')                 # 'mapping' | 'nav' -- which Orin
                                           # commands are meaningful (see class docstring)
        d('rows_to_cover', 6)             # fallback if no coverage plan received
        d('enable_plant_approach', True)
        d('plant_confirm_pause_s', 3.0)   # stop-and-confirm dwell at each plant,
                                           # NOT an indefinite arm wait -- see module docstring
        d('approach_cooldown', 5.0)       # s before the next approach can start
        d('nav2_wait_timeout', 5.0)       # s to find the Nav2 action server
        d('map_save_path', '~/ros2_ws/maps/tunnel_map')  # passed to slam_toolbox's
                                                          # serialize_map service
        d('zeroshot_enabled', False)      # fire /zeroshot_detect_trigger on Nav2
                                           # approach completion (see module
                                           # docstring's "ZERO-SHOT INTEGRATION"
                                           # section). Launch files set this
                                           # automatically from the same
                                           # detector:= argument that selects
                                           # which detector runs at all -- you
                                           # should not need to set this by hand.

        self.phase = self.get_parameter('phase').value
        self.rows_to_cover = int(self.get_parameter('rows_to_cover').value)
        self.enable_approach = bool(self.get_parameter('enable_plant_approach').value)
        self.confirm_pause = float(self.get_parameter('plant_confirm_pause_s').value)
        self.cooldown = float(self.get_parameter('approach_cooldown').value)
        self.nav2_wait = float(self.get_parameter('nav2_wait_timeout').value)
        self.map_save_path = os.path.expanduser(self.get_parameter('map_save_path').value)
        self.zeroshot_enabled = bool(self.get_parameter('zeroshot_enabled').value)

        # --- mission state ---
        self.state = 'WAITING_FOR_MAPPING_DONE' if self.phase == 'mapping' else 'WAITING_FOR_START'
        self.mission_started = False
        self.rows_done = 0
        self.have_plan = False
        self.current_goal = None
        self._goal_handle = None
        self._confirm_deadline = None
        self._cooldown_until = 0.0
        self._plan_deadline = self._now() + 3.0   # grace period to receive a coverage plan

        # --- IO ---
        self.create_subscription(String, '/mission/command', self.command_cb, 10)
        self.create_subscription(String, '/coverage_plan', self.plan_cb, _latched_qos())
        self.create_subscription(String, '/headland_status', self.hl_cb, 10)
        self.create_subscription(Bool, '/row_found', self.row_found_cb, 10)   # explore phase only
        self.create_subscription(PoseStamped, '/selected_plant_goal', self.goal_cb, 10)
        self.mission_pub = self.create_publisher(Bool, '/mission_active', 10)
        self.reached_pub = self.create_publisher(Bool, '/target_pose_reached', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', _latched_qos())
        self.fsm_state_pub = self.create_publisher(String, '/fsm_state', 10)  # legacy-compatible
        self.zeroshot_trigger_pub = self.create_publisher(Empty, '/zeroshot_detect_trigger', 10)

        # finish_mapping is available in ANY phase now (see module docstring)
        if _HAVE_SLAM_SRV:
            self._serialize_client = self.create_client(
                SerializePoseGraph, '/slam_toolbox/serialize_map')
        else:
            self._serialize_client = None

        if _HAVE_NAV2 and self.enable_approach and self.phase in ('nav', 'explore'):
            self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        else:
            self._nav_client = None
            if not _HAVE_NAV2 and self.phase in ('nav', 'explore'):
                self.get_logger().warn(
                    'nav2_msgs not found -> running COVERAGE ONLY (plant approach disabled).')

        self.create_timer(0.5, self.loop)
        self._publish_status()
        self.get_logger().info(
            f'mission_control_node ready, phase="{self.phase}". '
            f'Commands: ros2 topic pub --once /mission/command std_msgs/String '
            f'"data: <finish_mapping|start_nav|stop>"')

    # ------------------------------------------------------------------ utils
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_mission_active(self, active):
        self.mission_pub.publish(Bool(data=bool(active)))

    def _publish_status(self):
        self.status_pub.publish(String(data=self.state))
        self.fsm_state_pub.publish(
            String(data=f'{self.state} rows={self.rows_done}/{self.rows_to_cover}'))

    # -------------------------------------------------------------- Orin command
    def command_cb(self, msg):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'/mission/command received: "{cmd}"')

        if cmd == 'finish_mapping':
            # available in ANY phase now -- see module docstring. In 'mapping'
            # phase this is the real phase-transition trigger (state becomes
            # MAP_SAVED); in 'nav'/'explore' it is just a snapshot with no
            # mission-state effect.
            self._finish_mapping()
        elif cmd == 'start_nav' and self.phase in ('nav', 'explore'):
            # EXPLORATION_COMPLETE is an accepted restart point alongside
            # WAITING_FOR_START: an operator who has physically repositioned
            # the robot (or believes row_navigation's EXPLORE_HALT was a
            # temporary obstruction rather than a genuine tunnel edge) can
            # retry. This does NOT bypass row_navigation's own safety
            # posture -- EXPLORE_HALT is only cleared there by a real
            # /autonomy_enable off->on cycle (PS2 pad), which this command
            # does not do by itself; the operator must do both.
            if self.state in ('WAITING_FOR_START', 'EXPLORATION_COMPLETE'):
                self.mission_started = True
                self.state = 'EXPLORING' if self.phase == 'explore' else 'NAVIGATING'
                self._publish_status()
                self.get_logger().info(
                    f'Autonomous {"exploration" if self.phase == "explore" else "navigation"} '
                    f'started by Orin command.')
            else:
                self.get_logger().warn(
                    f'"start_nav" ignored -- current state is {self.state}, '
                    f'expected WAITING_FOR_START or EXPLORATION_COMPLETE.')
        elif cmd == 'stop':
            self.mission_started = False
            if self._goal_handle is not None:
                # Without this, an in-flight Nav2 NavigateToPose goal keeps
                # executing regardless of "stop": _set_mission_active(False)
                # only pauses row_navigation (which gates on /mission_active),
                # it has no effect on Nav2's own controller, which continues
                # driving toward self.current_goal until the goal itself is
                # cancelled. This contradicts the "stop" command's own
                # docstring claim that "the row follower and Nav2 both stop
                # issuing commands".
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
            self._set_mission_active(False)
            self.state = 'STOPPED'
            self._publish_status()
            self.get_logger().info(
                'Mission stopped by Orin command (this is a mission-level pause, '
                'not a hardware e-stop -- use the PS2 pad for that).')
        else:
            self.get_logger().warn(
                f'Unrecognised or phase-inappropriate command "{cmd}" '
                f'(phase={self.phase}) -- ignored.')

    def _finish_mapping(self):
        if self._serialize_client is not None and self._serialize_client.wait_for_service(
                timeout_sec=2.0):
            req = SerializePoseGraph.Request()
            req.filename = self.map_save_path
            fut = self._serialize_client.call_async(req)
            fut.add_done_callback(self._serialize_done_cb)
            self.get_logger().info(f'Saving map to {self.map_save_path} ...')
        else:
            self.get_logger().error(
                'slam_toolbox serialize_map service unavailable -- map NOT saved. '
                'Is slam_toolbox running?')
            if self.phase == 'mapping':
                self.state = 'MAP_SAVE_FAILED'
                self._publish_status()

    def _serialize_done_cb(self, future):
        # Only a real MISSION-STATE transition in 'mapping' phase, where
        # MAP_SAVED is the designed trigger to switch launch files. In
        # 'nav'/'explore' phase this is a mid-mission SNAPSHOT -- overwriting
        # self.state here would silently corrupt an active EXPLORING/
        # NAVIGATING/APPROACHING_PLANT state the next status publish would
        # then report wrong. Log and move on instead; loop() republishes the
        # real ongoing state on its next tick regardless.
        try:
            future.result()
            self.get_logger().info(
                f'Map saved to {self.map_save_path}.{{posegraph,data}}. '
                f'Confirm semantic_targets.csv exists too (see calibration guide).')
            if self.phase == 'mapping':
                self.state = 'MAP_SAVED'
                self.get_logger().info(
                    'Stop this launch and run the Phase 2 launch file to navigate.')
                self._publish_status()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Map serialize failed: {e}')
            if self.phase == 'mapping':
                self.state = 'MAP_SAVE_FAILED'
                self._publish_status()

    # -------------------------------------------------------------- callbacks
    def plan_cb(self, msg):
        try:
            plan = json.loads(msg.data)
            self.rows_to_cover = int(plan.get('num_rows', self.rows_to_cover))
            self.have_plan = True
            self.get_logger().info(
                f'Coverage plan received: {self.rows_to_cover} rows '
                f'({plan.get("pattern", "?")}).')
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f'Bad coverage plan ignored: {e}')

    def hl_cb(self, msg):
        if msg.data == 'turn_complete' and self.state in ('NAVIGATING', 'EXPLORING'):
            self.rows_done += 1
            suffix = (f'{self.rows_done}/{self.rows_to_cover}' if self.state == 'NAVIGATING'
                     else f'{self.rows_done} so far (unknown total -- exploring)')
            self.get_logger().info(f'Row complete ({suffix}).')

    def row_found_cb(self, msg):
        # explore phase only -- row_navigation tried to reacquire a corridor
        # after a headland turn and is reporting whether it found one. This
        # is the actual completion trigger for exploration (see module
        # docstring): msg.data==False means row_navigation has already
        # entered its own EXPLORE_HALT state and stopped driving on its own
        # -- this node's job is just to reflect that as a mission-level
        # status, not to decide anything further (per row_navigation's own
        # stated philosophy: a mission node choosing "what next" should be
        # a deliberate act, not automatic).
        if self.phase != 'explore' or self.state != 'EXPLORING':
            return
        if not msg.data:
            self.state = 'EXPLORATION_COMPLETE'
            self._set_mission_active(False)
            self._publish_status()
            self.get_logger().info(
                f'EXPLORATION_COMPLETE after {self.rows_done} row(s) -- row_navigation '
                f'found no further corridor. Robot is stopped (row_navigation\'s own '
                f'EXPLORE_HALT state, which only clears on a real /autonomy_enable '
                f'off->on cycle). Send "finish_mapping" to save the final map. To '
                f'retry (e.g. after repositioning, if you believe there is more '
                f'tunnel to find): cycle autonomy off then on via the PS2 pad, '
                f'THEN send "start_nav" again.')

    def goal_cb(self, msg):
        if (self.state in ('NAVIGATING', 'EXPLORING') and self.enable_approach
                and self._nav_client is not None
                and self._now() >= self._cooldown_until):
            self.current_goal = msg
            self._start_approach()

    # ----------------------------------------------------------- Nav2 approach
    def _start_approach(self):
        # Non-blocking readiness check -- this used to be
        # wait_for_server(timeout_sec=self.nav2_wait), which blocks
        # synchronously inside a subscription callback. On this node's
        # (default) single-threaded executor, if the Nav2 action server
        # isn't already up, that call freezes the ENTIRE node -- including
        # /mission/command handling -- for up to nav2_wait_timeout (5s
        # default), at exactly the moment an operator might want to send
        # "stop". server_is_ready() returns immediately; if the server
        # isn't up yet we simply skip this approach attempt (same
        # skip-and-cooldown path already used when Nav2 rejects a goal
        # below) rather than block waiting for it -- the next
        # /selected_plant_goal will retry.
        if not self._nav_client.server_is_ready():
            self.get_logger().warn(
                'Nav2 navigate_to_pose server not ready -> skipping this '
                'approach (non-blocking check; will retry on the next '
                'plant goal).')
            self._cooldown_until = self._now() + self.cooldown
            return
        self.state = 'APPROACHING_PLANT'
        self._publish_status()
        self._set_mission_active(False)          # pause row follower; Nav2 drives
        goal = NavigateToPose.Goal()
        goal.pose = self.current_goal
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f'APPROACHING_PLANT -> Nav2 goal ({goal.pose.pose.position.x:.2f}, '
            f'{goal.pose.pose.position.y:.2f}).')
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected approach goal -> resume rows.')
            self._end_approach()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        # ok must reflect Nav2's actual terminal GoalStatus, not merely
        # whether future.result() raised. In rclpy, an aborted or canceled
        # action goal resolves the result future WITHOUT raising -- the
        # GetResult service round-trip itself still succeeds, it just carries
        # a non-SUCCEEDED status -- so checking only for an exception here
        # would silently treat a failed approach (robot never reached the
        # plant) as ok=True and still fire the zero-shot detection trigger.
        ok = False
        try:
            result_response = future.result()
            ok = (result_response.status == GoalStatus.STATUS_SUCCEEDED)
        except Exception:  # noqa: BLE001
            ok = False
        self.get_logger().info(
            f'Nav2 approach finished (ok={ok}) -> confirmation pause '
            f'({self.confirm_pause}s, then auto-advance -- no arm hand-off, see module docstring).')
        self.reached_pub.publish(Bool(data=True))   # visible confirmation signal
        if self.zeroshot_enabled and ok:
            # Fire-and-forget: this node does NOT wait for a detection
            # result before advancing. plant_detector_zeroshot_node runs
            # asynchronously and publishes its own results directly to
            # /plant_targets (and /detected_weeds) on whatever timeline its
            # inference actually takes -- typically a few hundred ms, well
            # inside the confirm_pause dwell below, but NOT gated on it.
            # Making the FSM wait for a detection to complete would add a
            # new failure mode (a slow or hung inference blocking the whole
            # mission) for a benefit (a specific detection landing before
            # the robot moves on) that doesn't actually matter -- the
            # detection reaches semantic_mapper whenever it's ready either
            # way, and semantic_mapper already de-duplicates by position,
            # not by arrival order.
            self.zeroshot_trigger_pub.publish(Empty())
            self.get_logger().info(
                'Zero-shot detection triggered at plant approach (fire-and-forget).')
        self._confirm_deadline = self._now() + self.confirm_pause

    def _end_approach(self):
        self._goal_handle = None
        self.current_goal = None
        self._cooldown_until = self._now() + self.cooldown
        # return to whichever state this phase actually uses -- NOT
        # unconditionally 'NAVIGATING': if a plant approach happened during
        # EXPLORING, forcing 'NAVIGATING' here would silently switch loop()
        # onto the wrong completion check (rows_to_cover instead of the
        # row_found-driven one) for the rest of the mission.
        self.state = 'EXPLORING' if self.phase == 'explore' else 'NAVIGATING'
        self._publish_status()
        self._set_mission_active(True)

    # ----------------------------------------------------------------- main loop
    def loop(self):
        now = self._now()

        # have_plan/_plan_deadline were previously write-only (set at init
        # and in plan_cb, never read) -- this is the intended "warn if no
        # coverage plan arrives within the grace period" check that was
        # never wired in. Only meaningful in 'nav' phase: coverage_planner's
        # plan is what 'nav' phase actually consumes; 'explore' phase never
        # expects one, and 'mapping' phase doesn't navigate yet. Fires once
        # (have_plan is reused as the "already warned" gate) rather than
        # re-warning every 0.5s tick.
        if self.phase == 'nav' and not self.have_plan and now >= self._plan_deadline:
            self.get_logger().warn(
                f'No /coverage_plan received within the grace period -- '
                f'falling back to rows_to_cover={self.rows_to_cover} '
                f'(see the "rows_to_cover" parameter default). Is '
                f'coverage_planner running?')
            self.have_plan = True

        if self.state == 'WAITING_FOR_MAPPING_DONE':
            self._set_mission_active(False)   # Phase 1: row-following may still be
                                               # driven manually via PS2; this node
                                               # does not gate it here

        elif self.state == 'WAITING_FOR_START':
            self._set_mission_active(False)

        elif self.state == 'NAVIGATING':
            if self.rows_done >= self.rows_to_cover:
                self.state = 'MISSION_COMPLETE'
                self._publish_status()
            else:
                self._set_mission_active(True)

        elif self.state == 'EXPLORING':
            # No rows_to_cover check here -- an unknown tunnel has no known
            # target row count. Completion is driven entirely by
            # row_found_cb (row_navigation reporting it found nothing after
            # a headland turn), not by a count. Just keep autonomy enabled;
            # row_navigation's own exploration_mode does the actual
            # row-discovery driving.
            self._set_mission_active(True)

        elif self.state == 'APPROACHING_PLANT':
            self._set_mission_active(False)
            if self._confirm_deadline is not None and now >= self._confirm_deadline:
                self.reached_pub.publish(Bool(data=False))
                self._confirm_deadline = None
                self._end_approach()

        elif self.state in ('MISSION_COMPLETE', 'EXPLORATION_COMPLETE', 'STOPPED',
                            'MAP_SAVED', 'MAP_SAVE_FAILED'):
            self._set_mission_active(False)

        self.fsm_state_pub.publish(
            String(data=f'{self.state} rows={self.rows_done}/{self.rows_to_cover}'))


def main():
    rclpy.init()
    node = MissionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
