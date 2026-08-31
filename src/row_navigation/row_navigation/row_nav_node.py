#!/usr/bin/env python3
"""
Tabletop crop-row following + headland turning for the Ackermann (rear-drive,
front-steer) dual-Pi AMR (ROSMASTER R2 chassis: 2x JGB37-520 rear motors,
YB-P20M front steering servo).

Built for SUBSTRATE TABLETOP strawberry tunnels. At LiDAR height the scan sees
the vertical support POSTS on each side of the aisle (NOT a solid foliage
wall), so the corridor is estimated from the *inner boundary* of the posts:

  1. bin the forward scan in x (robot frame: x fwd, y left)
  2. in each bin take the innermost left post (smallest +y) and innermost
     right post (largest -y); the bin centre is their midpoint
  3. RANSAC-fit a line to the centre points -> slope = heading error,
     intercept = cross-track (lateral) offset
  4. steer with a Fractional-Order PID (FOPID) on the heading error plus a
     proportional cross-track term

THIS PART IS UNCHANGED FROM THE SKID-STEER DESIGN. RANSAC and FOPID reason
purely in terms of a desired (v, omega) Twist, which is steering-geometry
agnostic; the kinematic conversion to an Ackermann (speed, steering_angle) command
now happens downstream inside base_controller, via Yahboom's own
Rosmaster_Lib set_car_motion() call (car-type-aware, firmware-side --
see base_controller_node.py's module docstring), AFTER twist_mux
arbitration, so this node still just publishes Twist on /cmd_vel_auto.

WHAT *IS* NEW: headland turning. A skid-steer base can pivot on the spot
(v=0, omega!=0); an Ackermann base fundamentally cannot -- omega is only
producible while v!=0, and is bounded by the mechanical minimum turning
radius R_min = L / tan(delta_max). Because the tabletop row spacing
(~0.35-0.6 m) is well under the achievable turning DIAMETER (2*R_min ~= 0.73 m
for this chassis: L=0.25 m, delta_max=0.6 rad/34.4 deg -- L here is the
planning-layer wheelbase, deliberately rounded up from the true CAD/measured
0.2353 m (base_controller and both URDFs keep the real value; only this
node's kinematic model uses the rounder, more conservative one -- see
docs/HEADLAND_TURN_GEOMETRY.md and row_navigation_params.yaml's wheelbase
comment for why), a single arc cannot
land on the next row -- this node instead executes a verified two-arc
"bulb turn": straight exit buffer -> arc at +delta_max -> arc at -delta_max
(split angle solved once at startup so the two arcs land exactly on the next
row centreline with an exact 180 deg heading reversal) -> short reacquire
creep -> hand back to RANSAC+FOPID. Turn direction alternates each time.
See docs/HEADLAND_TURN_GEOMETRY.md for the derivation and verification.

Motion is commanded ONLY when BOTH gates are true:
  /autonomy_enable (Bool)  -- human master switch (PS2 deadman/toggle)
  /mission_active  (Bool)  -- high-level FSM (defaults True)

Publishes:
  /cmd_vel_auto        geometry_msgs/Twist   (v, omega -- NOT yet Ackermann;
                                               guarded by safety_supervisor,
                                               converted by base_controller
                                               via Rosmaster_Lib)
  /row_lateral_offset  std_msgs/Float32
  /row_heading_error    std_msgs/Float32
  /row_end_detected    std_msgs/Bool         -- diagnostic only (a simple
                                              boolean pulse of the same event
                                              /headland_status's 'row_end'
                                              state already carries); nothing
                                              in this workspace consumes it
  /headland_status     std_msgs/String  ('follow'|'row_end'|'exit_buffer'|
                                          'arc1'|'arc2'|'turn_complete'|
                                          'reacquire')

IMPORTANT: v is NEVER commanded to zero while omega!=0 in this node (that
combination is physically meaningless for Ackermann and would be silently
clamped/undefined by the bicycle-model conversion downstream). Stopping
(autonomy/mission gates false) always zeroes BOTH fields together.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32, Bool, String


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _arc_step(pose, kappa, phi):
    """Forward kinematics of one constant-curvature arc segment (the SINGLE
    source of truth for the bulb-turn geometry -- do not re-derive/inline
    this elsewhere; an earlier inlined "simplification" of this exact
    formula introduced a sign error that silently flipped the turn to the
    wrong side for some row spacings. Always call this primitive directly.)
    pose=(x,y,theta); kappa=signed curvature (+ => left turn); phi=signed
    arc angle swept (NOT arc length)."""
    x0, y0, th0 = pose
    th1 = th0 + phi
    x1 = x0 + (1.0 / kappa) * (math.sin(th1) - math.sin(th0))
    y1 = y0 - (1.0 / kappa) * (math.cos(th1) - math.cos(th0))
    return (x1, y1, th1)


def _bulb_turn_split(R, row_spacing, n=2000):
    """Solve, ONCE at startup, the two-arc 'bulb turn' split angle phi1 such
    that: arc1 (curvature +1/R, angle phi1) followed by arc2 (curvature
    -1/R, angle pi-phi1) lands the vehicle at lateral offset = row_spacing
    with an exact 180 deg heading reversal. Local search over phi1 in
    [0.05,179.95] deg with phi2=pi-phi1 (which always guarantees the net
    heading change is exactly pi, by construction); we just pick the split
    that lands the lateral offset on target. Verified offline to land
    within sub-mm to ~0.5mm of the target across the realistic tabletop
    row-spacing range (0.35-0.60 m) -- see docs/HEADLAND_TURN_GEOMETRY.md.
    Re-solved at startup (2000-point grid, <50ms, runs once) rather than
    hardcoding a derived constant, so it always tracks the configured
    row_spacing/R_min exactly if either is retuned.

    Returns (phi1, phi2, x_forward_extent) where x_forward_extent is the
    forward (headland) depth consumed by the two arcs alone (NOT including
    the straight exit-buffer or reacquire-creep margins -- callers must add
    those, plus a vehicle-length margin, when sizing real headland
    clearance).
    """
    kappa = 1.0 / R
    best = None
    for phi1_deg in np.linspace(0.05, 179.95, n):
        phi1 = math.radians(phi1_deg)
        phi2 = math.pi - phi1
        p0 = (0.0, 0.0, 0.0)
        p1 = _arc_step(p0, +kappa, phi1)
        p2 = _arc_step(p1, -kappa, phi2)
        err = abs(p2[1] - row_spacing)
        if best is None or err < best[0]:
            best = (err, phi1, phi2, max(p1[0], p2[0]))
    _, phi1, phi2, x_extent = best
    return phi1, phi2, x_extent


class RowNavNode(Node):
    def __init__(self):
        super().__init__('row_nav_node')
        d = self.declare_parameter
        # --- FOPID gains / orders ---
        # NOTE: these are CARRIED OVER from the skid-steer/0.20 m/s-creep
        # design. The new walking-pace nominal speed (~6x faster) changes
        # loop dynamics; treat these as a starting point for field retuning,
        # not as validated-at-this-speed values.
        d('Kp_theta', 1.2)
        d('Ki_theta', 0.25)
        d('Kd_theta', 0.30)
        d('fopid_lambda', 0.4)        # fractional INTEGRAL order  (lambda)
        d('fopid_mu', 0.7)            # fractional DERIVATIVE order (mu)
        d('gl_history_length', 20)    # Gruenwald-Letnikov memory window M
        d('k_cross_track', 0.8)       # cross-track (lateral) feedback gain
        # --- speeds (Ackermann / human-walking-pace regime) ---
        d('nominal_linear_speed', 1.20)   # "normal human walking pace"
        d('min_linear_speed', 0.30)       # Ackermann floor: v must stay >0
        d('max_angular_speed', 1.2)       # clamp BEFORE bicycle-model convert
        d('align_slowdown', 1.2)          # speed *= (1 - k*|heading_err|)
        # --- corridor geometry (m), base_link frame ---
        d('fwd_window_min', 0.20)
        d('fwd_window_max', 3.0)          # widened: faster travel -> look further ahead
        d('num_bins', 8)
        d('side_gate', 0.10)
        d('target_half_width', 0.50)
        # --- RANSAC on centre points ---
        d('ransac_iterations', 50)
        d('ransac_inlier_threshold', 0.05)
        d('ransac_min_points', 3)
        # --- row-end / headland (Ackermann two-arc bulb turn) ---
        d('row_end_min_side_points', 6)
        d('wheelbase', 0.25)               # L (m) -- planning-layer value, NOT the true CAD
                                           # 0.2353m -- see row_navigation_params.yaml's comment
                                           # and docs/HEADLAND_TURN_GEOMETRY.md
        d('max_steer_angle', 0.6)         # delta_max, CAD joint limit (rad, 34.4 deg)
        d('row_spacing', 0.50)            # MUST match coverage_planner's value
        d('headland_exit_buffer', 0.40)   # straight creep past row end before arcing (m)
        d('headland_turn_speed', 0.35)    # v during both arcs -- deliberately << v_nom
        d('headland_yaw_tol', 0.05)
        d('headland_reacquire_time', 1.5)
        d('headland_reacquire_speed', 0.30)
        d('odom_timeout', 1.0)             # s -- see _do_headland: if
                                            # /odometry/filtered goes stale
                                            # mid-headland-manoeuvre, halt
                                            # rather than loop on a frozen
                                            # yaw estimate
        d('exploration_mode', False)   # opt-in reactive row-discovery -- see
                                        # _do_headland's REACQUIRE block. When
                                        # False (default), behaviour is
                                        # UNCHANGED from before this parameter
                                        # existed: coverage_planner's own
                                        # pre-configured route is used, and
                                        # REACQUIRE always transitions to
                                        # FOLLOW unconditionally.
        d('alternate_turn_direction', True)
        # --- safety ---
        d('autostart', False)

        g = lambda n: self.get_parameter(n).value
        self.Kp, self.Ki, self.Kd = g('Kp_theta'), g('Ki_theta'), g('Kd_theta')
        self.lam, self.mu = g('fopid_lambda'), g('fopid_mu')
        self.M = int(g('gl_history_length'))
        self.k_ct = g('k_cross_track')
        self.v_nom, self.v_min = g('nominal_linear_speed'), g('min_linear_speed')
        self.w_max, self.slowk = g('max_angular_speed'), g('align_slowdown')
        self.fwd_min, self.fwd_max = g('fwd_window_min'), g('fwd_window_max')
        self.nbins, self.gate = int(g('num_bins')), g('side_gate')
        self.half_w = g('target_half_width')
        self.rs_iter = int(g('ransac_iterations'))
        self.rs_thr = g('ransac_inlier_threshold')
        self.rs_min = int(g('ransac_min_points'))
        self.end_min = int(g('row_end_min_side_points'))

        self.L = g('wheelbase')
        self.delta_max = g('max_steer_angle')
        self.row_spacing = g('row_spacing')
        self.exit_buffer = g('headland_exit_buffer')
        self.turn_v = g('headland_turn_speed')
        self.turn_tol = g('headland_yaw_tol')
        self.reacq_t = g('headland_reacquire_time')
        self.reacq_v = g('headland_reacquire_speed')
        self.odom_timeout = g('odom_timeout')
        self.explore = bool(g('exploration_mode'))
        self.alt_turn = g('alternate_turn_direction')

        # Refuse to start rather than silently compute a wrong or crashing
        # R_min from a misconfigured steer angle. delta_max=0.0 would raise
        # a raw ZeroDivisionError with no diagnostic pointing at the actual
        # cause; a negative or >=90 deg value would not crash at all, just
        # silently produce a meaningless R_min and downstream bulb-turn
        # geometry. Same "refuse to start on bad config" pattern already
        # applied to safety_supervisor_node.cpp's slow_distance/stop_distance
        # validation this session.
        if not (0.0 < self.delta_max < (math.pi / 2.0)):
            raise ValueError(
                f'row_nav_node: max_steer_angle ({self.delta_max}) must be strictly '
                f'between 0 and pi/2 radians (0 and 90 deg).')

        # Refuse to start if target_half_width violates the constraint this
        # config file's own comment already documents in words -- an earlier
        # incident (see row_navigation_params.yaml's comment on
        # target_half_width) had this defaulted equal to row_spacing, which
        # is physically meaningless (two adjacent rows' assumed post lines
        # sitting exactly on top of each other). A comment alone did not
        # prevent it happening once; a startup check does.
        if not (2.0 * self.half_w < self.row_spacing):
            raise ValueError(
                f'row_nav_node: 2*target_half_width ({2.0 * self.half_w}) must be '
                f'less than row_spacing ({self.row_spacing}) -- see '
                f'row_navigation_params.yaml\'s comment on target_half_width for why.')

        self.R_min = self.L / math.tan(self.delta_max)
        self.phi1, self.phi2, x_extent = _bulb_turn_split(self.R_min, self.row_spacing)
        depth_needed = self.exit_buffer + x_extent + 0.35  # +0.35 m vehicle-length margin
        self.get_logger().info(
            f'Ackermann bulb-turn solved at startup: R_min={self.R_min:.3f} m, '
            f'row_spacing={self.row_spacing:.2f} m -> phi1={math.degrees(self.phi1):.1f} deg, '
            f'phi2={math.degrees(self.phi2):.1f} deg. '
            f'Estimated headland depth REQUIRED: ~{depth_needed:.2f} m '
            f'(verify this is clear in the real tunnel before autonomous operation).')

        # GL fractional weights (precomputed once)
        self.w_mu = self._gl_weights(self.mu, self.M)
        self.w_lam = self._gl_weights(-self.lam, self.M)

        # state
        self.err_hist = []
        self.mode = 'IDLE'   # IDLE|FOLLOW|EXIT_BUFFER|ARC1|ARC2|REACQUIRE|
                              # EXPLORE_HALT|ODOM_LOST_HALT
        self._reacq_extended = False   # exploration-mode: has the bounded extra
                                        # reacquire attempt already been used this headland
        self.autonomy = bool(g('autostart'))
        self.mission = True
        self.last_t = None
        self.turn_sign = 1.0
        self.yaw = None
        self.robot_x = None    # map-frame position -- tracked ONLY for the
        self.robot_y = None    # /row_nav/assumed_path viz publish below;
                                # the control law itself is local (base_link-
                                # frame lateral/heading error) and never reads
                                # these two fields.
        self.last_odom_time = None
        self.yaw_at_phase_start = None
        self.exit_until = None
        self.reacq_until = None

        # io
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.pub_lat = self.create_publisher(Float32, '/row_lateral_offset', 10)
        self.pub_hed = self.create_publisher(Float32, '/row_heading_error', 10)
        self.pub_end = self.create_publisher(Bool, '/row_end_detected', 10)
        self.pub_st = self.create_publisher(String, '/headland_status', 10)
        self.pub_row_found = self.create_publisher(Bool, '/row_found', 10)   # exploration mode only
        self.pub_assumed_path = self.create_publisher(Path, '/row_nav/assumed_path', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 20)
        self.create_subscription(Bool, '/autonomy_enable', self.auton_cb, 10)
        self.create_subscription(Bool, '/mission_active', self.mission_cb, 10)
        self.get_logger().info(
            'row_nav_node ready (tabletop corridor + FOPID + Ackermann bulb-turn headland).')

    # ---------- Gruenwald-Letnikov fractional calculus (UNCHANGED) ----------
    @staticmethod
    def _gl_weights(alpha, n):
        w = [1.0]
        for j in range(1, n):
            w.append(w[-1] * (1.0 - (alpha + 1.0) / j))
        return np.asarray(w)

    def _gl_sum(self, hist, w):
        m = min(len(hist), len(w))
        if m == 0:
            return 0.0
        return float(np.dot(w[:m], np.asarray(hist[:m])))

    # ---------- callbacks ----------
    def auton_cb(self, m):
        self.autonomy = bool(m.data)
        if not self.autonomy:
            self.mode = 'IDLE'
            self.err_hist.clear()

    def mission_cb(self, m):
        self.mission = bool(m.data)

    def odom_cb(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robot_x = m.pose.pose.position.x
        self.robot_y = m.pose.pose.position.y
        self.last_odom_time = self.get_clock().now().nanoseconds * 1e-9

    def _status(self, s):
        self.pub_st.publish(String(data=s))

    def _stop(self):
        # v and omega ALWAYS zeroed together -- never publish omega!=0 with v=0
        self.pub_cmd.publish(Twist())

    def scan_cb(self, msg: LaserScan):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.1 if self.last_t is None else max(1e-3, now - self.last_t)
        self.last_t = now

        if self.mode in ('EXPLORE_HALT', 'ODOM_LOST_HALT'):
            # Checked BEFORE the autonomy/mission gate below, deliberately.
            # Stays here -- genuinely stopped -- until a mission node (or an
            # operator) cycles /autonomy_enable false then true, which
            # auton_cb already routes through 'IDLE' and back into normal
            # operation, independent of this ordering. Deliberately NOT
            # auto-cleared by anything else: EXPLORE_HALT exists specifically
            # so nothing resumes driving into unknown space on its own after
            # a "no row found" result, and ODOM_LOST_HALT exists so nothing
            # resumes a headland manoeuvre with an unknown true heading.
            #
            # This ordering matters: if the gate below ran first, it would
            # silently downgrade either halt to 'IDLE' -- which is not
            # specially handled anywhere else in this function and offers no
            # equivalent protection -- the instant /mission_active goes
            # False. For EXPLORE_HALT specifically, that is not a rare edge
            # case: mission_control_node.py's row_found_cb reacts to this
            # state's own /row_found=False publish by calling
            # _set_mission_active(False), which is the NORMAL, EXPECTED
            # sequence every single time this state is entered. Checking the
            # halt state first means the documented "stays stopped until a
            # real autonomy cycle" guarantee is actually enforced by this
            # state machine, not merely true by coincidence of mission_active
            # happening to stay False.
            self._stop()
            return

        if not (self.autonomy and self.mission):
            self._stop()
            self.mode = 'IDLE'
            return

        if self.mode in ('EXIT_BUFFER', 'ARC1', 'ARC2', 'REACQUIRE'):
            self._do_headland(now, msg)
            return

        # --- corridor estimate (UNCHANGED RANSAC core) ---
        xs, ys = self._scan_to_xy(msg)
        left = ys > self.gate
        right = ys < -self.gate
        nL, nR = int(np.sum(left)), int(np.sum(right))

        if nL < self.end_min and nR < self.end_min:
            self.pub_end.publish(Bool(data=True))
            self._begin_headland()
            return
        self.pub_end.publish(Bool(data=False))

        cx, cy = self._centre_points(xs, ys, left, right)
        if cx.size < 2:
            cmd = Twist()
            cmd.linear.x = self.v_min
            self.pub_cmd.publish(cmd)
            self._status('follow')
            return

        m_slope, b_int = self._ransac_line(cx, cy)
        heading_err = math.atan(m_slope)
        lateral = b_int

        self._publish_assumed_path(m_slope, b_int)

        # --- FOPID on heading error (UNCHANGED) ---
        self.err_hist.insert(0, heading_err)
        if len(self.err_hist) > self.M:
            self.err_hist = self.err_hist[:self.M]
        d_mu = (dt ** (-self.mu)) * self._gl_sum(self.err_hist, self.w_mu)
        i_lam = (dt ** (self.lam)) * self._gl_sum(self.err_hist, self.w_lam)
        omega = (self.Kp * heading_err
                 + self.Ki * i_lam
                 + self.Kd * d_mu
                 + self.k_ct * lateral)
        omega = float(np.clip(omega, -self.w_max, self.w_max))

        speed = max(self.v_min, self.v_nom * (1.0 - self.slowk * abs(heading_err)))
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = omega
        self.pub_cmd.publish(cmd)
        self.pub_lat.publish(Float32(data=float(lateral)))
        self.pub_hed.publish(Float32(data=float(heading_err)))
        self.mode = 'FOLLOW'
        self._status('follow')

    # ---------- visualization only (r2_web_viz) ----------
    def _publish_assumed_path(self, m_slope, b_int):
        """Project the locally-fitted corridor centerline into the map frame
        and publish it as the 'assumed trajectory' for r2_web_viz.

        Read-only: does not feed back into heading_err/lateral/omega/speed
        above. Row-following has no pre-planned global path -- it is a
        purely reactive controller tracking a line re-fit from the current
        scan every cycle -- so this is deliberately a short lookahead
        segment along that line, not a long-horizon plan. Skipped silently
        until the first /odometry/filtered message arrives.
        """
        if self.robot_x is None or self.robot_y is None or self.yaw is None:
            return
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)
        for i in range(9):
            x_local = self.fwd_max * i / 8.0
            y_local = m_slope * x_local + b_int
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = self.robot_x + x_local * cos_y - y_local * sin_y
            ps.pose.position.y = self.robot_y + x_local * sin_y + y_local * cos_y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_assumed_path.publish(path)

    # ---------- geometry (UNCHANGED) ----------
    def _scan_to_xy(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment
        ok = np.isfinite(r) & (r > max(0.05, msg.range_min)) & (r < msg.range_max)
        r, a = r[ok], a[ok]
        x, y = r * np.cos(a), r * np.sin(a)
        win = (x > self.fwd_min) & (x < self.fwd_max)
        return x[win], y[win]

    def _centre_points(self, xs, ys, left, right):
        edges = np.linspace(self.fwd_min, self.fwd_max, self.nbins + 1)
        cxs, cys = [], []
        for k in range(self.nbins):
            sel = (xs >= edges[k]) & (xs < edges[k + 1])
            _, ly = xs[sel & left], ys[sel & left]
            _, ry = xs[sel & right], ys[sel & right]
            bx = 0.5 * (edges[k] + edges[k + 1])
            if ly.size and ry.size:
                cys.append(0.5 * (np.min(ly) + np.max(ry)))
                cxs.append(bx)
            elif ly.size:
                cys.append(np.min(ly) - self.half_w)
                cxs.append(bx)
            elif ry.size:
                cys.append(np.max(ry) + self.half_w)
                cxs.append(bx)
        return np.asarray(cxs), np.asarray(cys)

    def _ransac_line(self, cx, cy):
        n = cx.size
        if n < max(self.rs_min, 2):
            m, b = np.polyfit(cx, cy, 1)
            return float(m), float(b)
        best_inl, best = -1, None
        for _ in range(self.rs_iter):
            i, j = np.random.choice(n, 2, replace=False)
            if abs(cx[j] - cx[i]) < 1e-6:
                continue
            m = (cy[j] - cy[i]) / (cx[j] - cx[i])
            b = cy[i] - m * cx[i]
            res = np.abs(cy - (m * cx + b))
            mask = res < self.rs_thr
            inl = int(np.sum(mask))
            if inl > best_inl:
                best_inl, best = inl, mask
        if best is None or best.sum() < 2:
            m, b = np.polyfit(cx, cy, 1)
        else:
            m, b = np.polyfit(cx[best], cy[best], 1)
        return float(m), float(b)

    # ---------- headland turn: verified two-arc Ackermann "bulb turn" ----------
    # CORRECTED MANEUVER (verified by closed-loop simulation against the
    # discrete state-machine logic below, including the exact yaw-threshold
    # switching). The invariant is the COMMANDED YAW RATE omega, not the
    # physical steering angle: ARC1 is FORWARD with omega=+omega_const*
    # turn_sign; ARC2 is REVERSE with omega held at that SAME value and sign.
    # Because base_controller's set_car_motion() call performs the
    # car-type-aware Ackermann conversion downstream (Rosmaster_Lib,
    # theta_dot == omega EXACTLY regardless of the sign of v (atan/tan are
    # inverses) -- so holding omega constant is what keeps the heading
    # sweeping in the SAME rotational direction across both arcs, summing to
    # exactly pi. The PHYSICAL steering angle sent to the servo is NOT held
    # constant -- it flips sign automatically between the two arcs, because
    # v flips while omega doesn't. That is correct and intentional (the same
    # forward-left/reverse-with-the-same-wheel-lock technique used in a real
    # 3-point turn), not a bug. Reasoning about this in terms of "the
    # steering lock" instead of omega is exactly how an earlier draft of
    # this comment (and of docs/HEADLAND_TURN_GEOMETRY.md) got the
    # explanation backwards -- do not change this sign convention, or
    # describe it in terms of delta, without re-reading that document's
    # worked derivation first.
    def _begin_headland(self):
        if self.yaw is None:
            self._stop()
            self._status('row_end')
            self.get_logger().warn('Row end but no odometry yaw yet; holding.')
            return
        self.yaw_at_phase_start = self.yaw
        self.exit_until = self.get_clock().now().nanoseconds * 1e-9 + (
            self.exit_buffer / max(self.turn_v, 1e-3))
        self.mode = 'EXIT_BUFFER'
        self.err_hist.clear()
        self._status('row_end')
        self.get_logger().info(f'Row end -> headland bulb-turn, sign={self.turn_sign:+.0f}')

    def _do_headland(self, now, msg):
        cmd = Twist()
        if self.yaw is None:
            self._stop()
            return
        if self.last_odom_time is None or (now - self.last_odom_time) > self.odom_timeout:
            # ARC1/ARC2's phase-advance condition (abs(err) < self.turn_tol)
            # is computed entirely from self.yaw, which is only ever updated
            # in odom_cb -- if /odometry/filtered stops arriving mid-arc,
            # self.yaw freezes and that condition can never become true
            # again, so this node would otherwise keep publishing that arc's
            # fixed nonzero (v, omega) command indefinitely. safety_supervisor
            # does not catch this: this node keeps publishing /cmd_vel_auto
            # on schedule (driven by /scan, not by odom), so its comms
            # watchdog never trips -- only the STATE feeding the decision is
            # stale, not the publish itself. EXIT_BUFFER/REACQUIRE do not
            # depend on self.yaw for their own advance condition (wall-clock
            # deadlines only) so are not vulnerable to looping forever, but
            # are halted here too rather than continuing to drive blind
            # through a turn with an unknown true heading.
            if self.mode != 'ODOM_LOST_HALT':
                self.get_logger().error(
                    f'/odometry/filtered stale (>{self.odom_timeout:.1f}s) during '
                    f'headland manoeuvre (was in {self.mode}) -- halting. Requires '
                    f'an explicit /autonomy_enable off->on cycle to resume, same '
                    f'as EXPLORE_HALT.')
            self.mode = 'ODOM_LOST_HALT'
            self._stop()
            return

        # yaw rate held at constant sign/magnitude through BOTH arcs; only
        # linear.x (forward vs reverse) and the resulting bicycle-model
        # steering angle (computed downstream, firmware-side, by
        # Rosmaster_Lib's set_car_motion() from the SIGN of linear.x --
        # see base_controller_node.py) change between the two phases.
        omega_const = (self.turn_v / self.R_min) * (1.0 if self.turn_sign > 0 else -1.0)

        if self.mode == 'EXIT_BUFFER':
            # straight creep clear of the last post before committing to the arc
            cmd.linear.x = self.turn_v
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            self._status('exit_buffer')
            if now >= self.exit_until:
                self.yaw_at_phase_start = self.yaw
                self.mode = 'ARC1'
            return

        if self.mode == 'ARC1':
            target = wrap(self.yaw_at_phase_start + self.turn_sign * self.phi1)
            err = wrap(target - self.yaw)
            cmd.linear.x = self.turn_v          # FORWARD
            cmd.angular.z = omega_const
            self.pub_cmd.publish(cmd)
            self._status('arc1')
            if abs(err) < self.turn_tol:
                self.yaw_at_phase_start = self.yaw
                self.mode = 'ARC2'
            return

        if self.mode == 'ARC2':
            target = wrap(self.yaw_at_phase_start + self.turn_sign * self.phi2)
            err = wrap(target - self.yaw)
            cmd.linear.x = -self.turn_v         # REVERSE -- see note above
            cmd.angular.z = omega_const          # SAME sign as ARC1, not flipped
            self.pub_cmd.publish(cmd)
            self._status('arc2')
            if abs(err) < self.turn_tol:
                if self.alt_turn:
                    self.turn_sign *= -1.0
                self.reacq_until = now + self.reacq_t
                self._reacq_extended = False   # fresh allowance for this headland turn
                self.mode = 'REACQUIRE'
                self._status('turn_complete')
                self.get_logger().info('Headland bulb-turn complete -> reacquire.')
            return

        if self.mode == 'REACQUIRE':
            cmd.linear.x = self.reacq_v
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            self._status('reacquire')
            if now >= self.reacq_until:
                # ---- OPT-IN exploration-mode outcome check ----
                # Existing verified behaviour (self.explore == False, the
                # default) is UNCHANGED below: unconditional transition to
                # FOLLOW, exactly as before this addition -- do not touch
                # that path. This block only runs for callers that have
                # explicitly asked for reactive row-discovery
                # (coverage_planner's own pre-configured route is still the
                # right choice whenever the layout IS known in advance).
                if self.explore:
                    xs, ys = self._scan_to_xy(msg)
                    left = ys > self.gate
                    right = ys < -self.gate
                    nL, nR = int(np.sum(left)), int(np.sum(right))
                    found = nL >= self.end_min or nR >= self.end_min
                    if found:
                        self.pub_row_found.publish(Bool(data=True))
                        self.mode = 'FOLLOW'
                    elif not self._reacq_extended:
                        # one bounded extra creep before giving up -- a row
                        # entrance can legitimately be a bit further than
                        # reacq_t alone covers; this is NOT an unbounded
                        # search, just a single fixed-size second attempt
                        self._reacq_extended = True
                        self.reacq_until = now + self.reacq_t
                        self.get_logger().info(
                            'REACQUIRE (exploration mode): no corridor found '
                            'yet, one extended attempt before declaring no '
                            'row here.')
                    else:
                        self.pub_row_found.publish(Bool(data=False))
                        self._status('no_row_found')
                        self._stop()
                        # NOT 'IDLE' -- that name is already used elsewhere in
                        # this file to mean "autonomy/mission disabled", and
                        # is re-entered from normal corridor-following on the
                        # very next scan whenever autonomy+mission are still
                        # true (see the top of scan_cb) -- reusing it here
                        # would NOT actually stay stopped, it would silently
                        # fall through into normal FOLLOW logic on the next
                        # callback. EXPLORE_HALT is a distinct state, checked
                        # explicitly at the top of scan_cb, that genuinely
                        # stays stopped until a mission node clears it.
                        self.mode = 'EXPLORE_HALT'
                        self.get_logger().warn(
                            'REACQUIRE (exploration mode): no corridor found '
                            'after extended attempt -- stopping, no row here. '
                            'A higher-level mission node must decide what to '
                            'do next (declare exploration complete, try a '
                            'different heading, etc.) -- this node will not '
                            'blindly keep driving into unknown space.')
                else:
                    self.mode = 'FOLLOW'
            return


def main():
    rclpy.init()
    node = RowNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
