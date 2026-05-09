"""
dwa_planner.py
==============
Forward-biased Dynamic Window Approach (DWA) local planner for greenhouse
corridor flight.

This version is tuned for safer real-drone testing:

  - It does NOT immediately enter recovery after one failed DWA sample.
  - It requires several consecutive BLOCKED cycles before recovery.
  - Recovery yaw is limited to small offsets around the current commanded row
    heading, instead of scanning arbitrary global directions.
  - It checks occupancy grid freshness; stale grid means no motion command.
  - It publishes useful debug status for clearance and blockage diagnosis.

Behavior:
  Normal:
    Evaluate forward/lateral velocity samples using a precomputed Euclidean
    Distance Transform of the occupancy grid. Select the best collision-free
    trajectory based on goal progress, clearance, speed, heading alignment,
    smoothness, and lateral penalty.

  Blocked:
    Hold position and count consecutive blocked cycles.

  Recovery:
    Try small yaw offsets around the row heading:
      right side priority: +10°, +20°, -10°, -20°
      left side priority : -10°, -20°, +10°, +20°

Topics:
  SUB  /fmu/out/vehicle_local_position    px4_msgs/VehicleLocalPosition
  SUB  /local_occupancy_grid              nav_msgs/OccupancyGrid
  SUB  /mission/goal                      px4_msgs/TrajectorySetpoint

  PUB  /fmu/in/trajectory_setpoint        px4_msgs/TrajectorySetpoint
  PUB  /dwa/status                        std_msgs/String
"""

import math

import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from std_msgs.msg import String


class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        # ── Motion constraints ────────────────────────────────────── #
        # Conservative defaults for indoor / greenhouse real testing.
        self.declare_parameter('max_fwd_speed', 0.30)
        self.declare_parameter('max_lat_speed', 0.12)
        self.declare_parameter('max_accel', 0.25)
        self.declare_parameter('cruise_speed', 0.22)
        self.declare_parameter('takeoff_height', 1.50)

        # ── DWA sampling ─────────────────────────────────────────── #
        self.declare_parameter('v_samples', 7)
        self.declare_parameter('vy_samples', 5)
        self.declare_parameter('sim_time', 1.8)
        self.declare_parameter('sim_steps', 18)

        # ── Scoring weights ──────────────────────────────────────── #
        self.declare_parameter('w_goal', 3.5)
        self.declare_parameter('w_clearance', 1.8)
        self.declare_parameter('w_speed', 0.6)
        self.declare_parameter('w_smooth', 1.0)
        self.declare_parameter('w_lateral', 2.8)
        self.declare_parameter('w_heading', 1.8)

        # ── Safety ───────────────────────────────────────────────── #
        # These are deliberately less conservative than before because
        # occupancy_grid already inflates obstacles.
        self.declare_parameter('hard_stop_dist', 0.35)
        self.declare_parameter('slow_zone_dist', 1.00)
        self.declare_parameter('goal_tolerance', 0.25)

        # Grid freshness
        self.declare_parameter('grid_timeout_sec', 0.70)

        # ── Recovery yaw ─────────────────────────────────────────── #
        self.declare_parameter('enable_recovery_yaw', True)
        self.declare_parameter('stuck_time_sec', 2.5)
        self.declare_parameter('min_progress_m', 0.04)

        # Recovery is limited to small yaw offsets around commanded_yaw.
        # right -> [10,20] means: +10, +20, -10, -20
        # left  -> [10,20] means: -10, -20, +10, +20
        self.declare_parameter('recovery_primary_side', 'right')
        self.declare_parameter('recovery_yaw_levels_deg', [10.0, 20.0])

        self.declare_parameter('recovery_yaw_tol_deg', 5.0)
        self.declare_parameter('recovery_hold_max_sec', 1.2)
        self.declare_parameter('recovery_cooldown_sec', 1.0)
        self.declare_parameter('recovery_scan_dist_m', 2.0)
        self.declare_parameter('recovery_step_m', 0.10)

        # Need consecutive blocked cycles before recovery to avoid reacting
        # to one-frame depth/grid noise.
        self.declare_parameter('blocked_count_before_recovery', 8)

        self.V_MAX = float(self.get_parameter('max_fwd_speed').value)
        self.VY_MAX = float(self.get_parameter('max_lat_speed').value)
        self.A_MAX = float(self.get_parameter('max_accel').value)
        self.V_CRUISE = float(self.get_parameter('cruise_speed').value)
        self.ALT = float(self.get_parameter('takeoff_height').value)

        self.V_SMP = int(self.get_parameter('v_samples').value)
        self.VY_SMP = int(self.get_parameter('vy_samples').value)
        self.SIM_T = float(self.get_parameter('sim_time').value)
        self.SIM_S = int(self.get_parameter('sim_steps').value)

        self.W_G = float(self.get_parameter('w_goal').value)
        self.W_C = float(self.get_parameter('w_clearance').value)
        self.W_S = float(self.get_parameter('w_speed').value)
        self.W_SM = float(self.get_parameter('w_smooth').value)
        self.W_LAT = float(self.get_parameter('w_lateral').value)
        self.W_H = float(self.get_parameter('w_heading').value)

        self.HARD_STOP = float(self.get_parameter('hard_stop_dist').value)
        self.SLOW_ZONE = float(self.get_parameter('slow_zone_dist').value)
        self.GOAL_TOL = float(self.get_parameter('goal_tolerance').value)
        self.GRID_TIMEOUT = float(self.get_parameter('grid_timeout_sec').value)

        self.EN_REC = bool(self.get_parameter('enable_recovery_yaw').value)
        self.STUCK_T = float(self.get_parameter('stuck_time_sec').value)
        self.MIN_PROGRESS = float(self.get_parameter('min_progress_m').value)
        self.BLOCKED_BEFORE_REC = int(
            self.get_parameter('blocked_count_before_recovery').value
        )

        self.REC_PRIMARY_SIDE = str(
            self.get_parameter('recovery_primary_side').value
        ).lower()

        self.REC_LEVELS_DEG = [
            abs(float(x)) for x in self.get_parameter('recovery_yaw_levels_deg').value
        ]

        if len(self.REC_LEVELS_DEG) < 2:
            self.REC_LEVELS_DEG = [10.0, 20.0]

        self.REC_YAW_TOL = math.radians(
            float(self.get_parameter('recovery_yaw_tol_deg').value)
        )
        self.REC_HOLD_MAX = float(self.get_parameter('recovery_hold_max_sec').value)
        self.REC_COOLDOWN = float(self.get_parameter('recovery_cooldown_sec').value)
        self.REC_SCAN_DIST = float(self.get_parameter('recovery_scan_dist_m').value)
        self.REC_STEP = float(self.get_parameter('recovery_step_m').value)

        # State
        self._dist_transform = None
        self.occupied_ratio = 0.0

        self.pos = [0.0, 0.0, 0.0]
        self.vel_world = [0.0, 0.0]
        self.yaw = 0.0

        self.goal = None
        self.grid = None
        self.last_grid_time = None

        self.last_cmd = [0.0, 0.0]

        self.active = False
        self.blocked_cnt = 0
        self.have_pos = False
        self.last_wait_reason = None

        # FSM
        self.mode = 'FORWARD'  # FORWARD / RECOVERY_YAW
        self.commanded_yaw = None
        self.recovery_target_yaw = None
        self.recovery_start_time = None
        self.recovery_cooldown_until = 0.0

        # Progress tracking
        self.last_progress_check_time = None
        self.last_progress_dist = None

        # QoS
        px4_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        px4_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.pos_cb,
            px4_sub_qos
        )

        self.create_subscription(
            OccupancyGrid,
            '/local_occupancy_grid',
            self.grid_cb,
            10
        )

        self.create_subscription(
            TrajectorySetpoint,
            '/mission/goal',
            self.goal_cb,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_pub_qos
        )

        self.status_pub = self.create_publisher(String, '/dwa/status', 10)

        self.create_timer(0.10, self.plan)

        self.get_logger().info(
            'DWA Planner ready | conservative greenhouse mode | '
            f'vmax={self.V_MAX:.2f} cruise={self.V_CRUISE:.2f} '
            f'hard_stop={self.HARD_STOP:.2f} recovery={self.REC_PRIMARY_SIDE}'
            f'{self.REC_LEVELS_DEG}'
        )

    # ─── Callbacks ───────────────────────────────────────────────── #
    def pos_cb(self, msg: VehicleLocalPosition):
        self.pos = [float(msg.x), float(msg.y), float(msg.z)]
        self.vel_world = [float(msg.vx), float(msg.vy)]
        self.yaw = float(msg.heading)
        self.have_pos = True

    def grid_cb(self, msg: OccupancyGrid):
        self.grid = msg
        self.last_grid_time = self.get_clock().now()

        arr = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height,
            msg.info.width
        )

        occ = arr > 50
        self.occupied_ratio = float(np.count_nonzero(occ)) / float(arr.size + 1e-6)

        # EDT: distance from each free cell to nearest occupied cell in metres.
        # Occupied cells get 0.
        self._dist_transform = distance_transform_edt(arr <= 50) * msg.info.resolution

    def goal_cb(self, msg: TrajectorySetpoint):
        gx, gy = msg.position[0], msg.position[1]

        if math.isnan(gx) or math.isnan(gy):
            return

        new_goal = [float(gx), float(gy)]

        if self.goal != new_goal:
            self.get_logger().info(f'New goal: ({gx:.2f}, {gy:.2f})')

            self.goal = new_goal
            self.active = True
            self.blocked_cnt = 0

            self.mode = 'FORWARD'
            self.commanded_yaw = self.yaw
            self.recovery_target_yaw = None
            self.recovery_start_time = None
            self.recovery_cooldown_until = 0.0

            self.last_progress_check_time = None
            self.last_progress_dist = None
            self.last_cmd = [0.0, 0.0]

    # ─── Main planning loop ──────────────────────────────────────── #
    def plan(self):
        reason = None

        if not self.have_pos:
            reason = 'WAIT_POS'
        elif self.goal is None:
            reason = 'WAIT_GOAL'
        elif not self.active:
            reason = 'INACTIVE'
        elif self.grid is None:
            reason = 'WAIT_GRID'
        elif self._age_sec(self.last_grid_time) > self.GRID_TIMEOUT:
            reason = f'STALE_GRID age={self._age_sec(self.last_grid_time):.2f}s'

        if reason is not None:
            if reason != self.last_wait_reason:
                self.get_logger().warn(f'DWA not publishing: {reason}')
                self.last_wait_reason = reason
            return

        self.last_wait_reason = None

        dist = self._dist_to_goal()

        if dist < self.GOAL_TOL:
            self._publish_vel_world(0.0, 0.0, self.commanded_yaw)
            self.active = False
            self.mode = 'FORWARD'
            self._pub_status('GOAL_REACHED')
            self.get_logger().info(
                f'Goal reached at ({self.pos[0]:.2f}, {self.pos[1]:.2f})'
            )
            return

        if self.commanded_yaw is None:
            self.commanded_yaw = self.yaw

        if self.mode == 'RECOVERY_YAW':
            self._run_recovery_yaw()
            return

        self._run_forward(dist)

    # ─── Forward mode ────────────────────────────────────────────── #
    def _run_forward(self, dist: float):
        speed_cap = self._adaptive_speed(dist)
        dw = self._dynamic_window(speed_cap)
        best = self._best_sample(dw)

        front_clear = self._directional_clearance(
            self.commanded_yaw,
            self.REC_SCAN_DIST
        )
        cur_clear = self._clearance_at(self.pos[0], self.pos[1])

        stuck = self._update_stuck_state(dist)
        if stuck:
            self.get_logger().warn(
                f'STUCK detected: dist={dist:.2f}, '
                f'cur_clear={cur_clear:.2f}, front_clear={front_clear:.2f}, '
                f'cmd_yaw_deg={math.degrees(self.commanded_yaw):.1f}'
            )

        now = self._now_sec()

        if best is None:
            self.blocked_cnt += 1
            self._publish_vel_world(0.0, 0.0, self.commanded_yaw)

            self._pub_status(
                f'BLOCKED cnt={self.blocked_cnt} '
                f'cur_clr={cur_clear:.2f} front_clr={front_clear:.2f} '
                f'occ={self.occupied_ratio:.2f} hard_stop={self.HARD_STOP:.2f} '
                f'speed_cap={speed_cap:.2f}'
            )

            if self.blocked_cnt < self.BLOCKED_BEFORE_REC:
                return

            if self.EN_REC and now >= self.recovery_cooldown_until:
                self._start_recovery('NO_FEASIBLE_TRAJ')
                return

            return

        if self.EN_REC and stuck and now >= self.recovery_cooldown_until:
            self._publish_vel_world(0.0, 0.0, self.commanded_yaw)
            self._start_recovery('STUCK')
            return

        self.blocked_cnt = 0

        v_fwd, v_lat = best
        self.last_cmd = [v_fwd, v_lat]

        vx_w, vy_w = self._body_to_world(v_fwd, v_lat, self.commanded_yaw)
        self._publish_vel_world(vx_w, vy_w, self.commanded_yaw)

        self._pub_status(
            f'OK mode=FORWARD vf={v_fwd:.2f} vl={v_lat:.2f} '
            f'vx={vx_w:.2f} vy={vy_w:.2f} dist={dist:.2f} '
            f'cur_clr={cur_clear:.2f} front_clr={front_clear:.2f} '
            f'occ={self.occupied_ratio:.2f} speed_cap={speed_cap:.2f}'
        )

    # ─── Greenhouse-safe recovery ────────────────────────────────── #
    def _find_best_recovery_yaw(self) -> float | None:
        """
        Greenhouse-safe recovery:
        only try small yaw offsets around the current commanded row heading.
        This avoids random global turns such as 90 deg / 120 deg.
        """
        base = self.commanded_yaw if self.commanded_yaw is not None else self.yaw

        levels = self.REC_LEVELS_DEG
        if self.REC_PRIMARY_SIDE == 'left':
            offsets_deg = [-levels[0], -levels[1], levels[0], levels[1]]
        else:
            offsets_deg = [levels[0], levels[1], -levels[0], -levels[1]]

        goal_dx = self.goal[0] - self.pos[0]
        goal_dy = self.goal[1] - self.pos[1]
        goal_norm = math.hypot(goal_dx, goal_dy) + 1e-6

        best_score = -np.inf
        best_yaw = None

        for off_deg in offsets_deg:
            yaw = self._wrap_angle(base + math.radians(off_deg))
            clr = self._directional_clearance(yaw, self.REC_SCAN_DIST)

            if clr < self.HARD_STOP:
                continue

            alignment = (
                math.cos(yaw) * goal_dx + math.sin(yaw) * goal_dy
            ) / goal_norm

            score = 0.7 * (clr / self.REC_SCAN_DIST) + 0.3 * alignment

            if score > best_score:
                best_score = score
                best_yaw = yaw

        return best_yaw

    def _start_recovery(self, reason: str):
        best_yaw = self._find_best_recovery_yaw()

        if best_yaw is None:
            self.get_logger().warn(
                f'Recovery ({reason}): no safe small-angle direction found, holding.'
            )
            self._pub_status('RECOVERY_FAIL')
            self.recovery_cooldown_until = self._now_sec() + self.REC_COOLDOWN
            return

        self.recovery_target_yaw = best_yaw
        self.recovery_start_time = self._now_sec()
        self.mode = 'RECOVERY_YAW'

        self.get_logger().warn(
            f'Recovery ({reason}): small-angle steering to '
            f'{math.degrees(best_yaw):.1f} deg '
            f'(current={math.degrees(self.yaw):.1f}, '
            f'cmd={math.degrees(self.commanded_yaw):.1f})'
        )

        self._pub_status(
            f'RECOVERY_START target_yaw_deg={math.degrees(best_yaw):.1f}'
        )

    def _run_recovery_yaw(self):
        if self.recovery_target_yaw is None:
            self.mode = 'FORWARD'
            return

        yaw_err = self._wrap_angle(self.recovery_target_yaw - self.yaw)
        elapsed = self._now_sec() - self.recovery_start_time

        self._publish_vel_world(0.0, 0.0, self.recovery_target_yaw)

        self._pub_status(
            f'RECOVERY_YAW err_deg={math.degrees(yaw_err):.1f} '
            f'target_deg={math.degrees(self.recovery_target_yaw):.1f}'
        )

        if abs(yaw_err) < self.REC_YAW_TOL:
            self.commanded_yaw = self.recovery_target_yaw
            self.recovery_target_yaw = None
            self.recovery_start_time = None
            self.recovery_cooldown_until = self._now_sec() + self.REC_COOLDOWN
            self.mode = 'FORWARD'
            self.blocked_cnt = 0
            self.last_progress_check_time = None
            self.last_progress_dist = None

            self.get_logger().info(
                f'Recovery yaw reached. Resume FORWARD at '
                f'{math.degrees(self.commanded_yaw):.1f} deg'
            )
            return

        if elapsed > self.REC_HOLD_MAX:
            # Do not keep waiting forever. Accept the small yaw target as the
            # new commanded_yaw, but this target is still only a small offset.
            self.commanded_yaw = self.recovery_target_yaw
            self.recovery_target_yaw = None
            self.recovery_start_time = None
            self.recovery_cooldown_until = self._now_sec() + self.REC_COOLDOWN
            self.mode = 'FORWARD'
            self.blocked_cnt = 0
            self.last_progress_check_time = None
            self.last_progress_dist = None

            self.get_logger().warn(
                'Recovery yaw timeout. Continuing with small-angle commanded yaw.'
            )

    # ─── Adaptive speed ──────────────────────────────────────────── #
    def _adaptive_speed(self, dist_to_goal: float) -> float:
        clearance = self._clearance_at(self.pos[0], self.pos[1])

        if clearance <= self.HARD_STOP:
            obs_factor = 0.0
        elif clearance < self.SLOW_ZONE:
            t = (clearance - self.HARD_STOP) / max(
                1e-6,
                self.SLOW_ZONE - self.HARD_STOP
            )
            obs_factor = max(0.15, t)
        else:
            obs_factor = 1.0

        goal_factor = min(1.0, dist_to_goal / 2.0)
        return self.V_CRUISE * min(obs_factor, goal_factor)

    # ─── Coordinate helpers ──────────────────────────────────────── #
    def _world_to_body(self, vx_w: float, vy_w: float, yaw: float):
        c = math.cos(yaw)
        s = math.sin(yaw)

        v_fwd = c * vx_w + s * vy_w
        v_lat = -s * vx_w + c * vy_w

        return v_fwd, v_lat

    def _body_to_world(self, v_fwd: float, v_lat: float, yaw: float):
        c = math.cos(yaw)
        s = math.sin(yaw)

        vx_w = c * v_fwd - s * v_lat
        vy_w = s * v_fwd + c * v_lat

        return vx_w, vy_w

    @staticmethod
    def _wrap_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def _dist_to_goal(self) -> float:
        return math.hypot(
            self.pos[0] - self.goal[0],
            self.pos[1] - self.goal[1]
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _age_sec(self, stamp) -> float:
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    # ─── Dynamic window in BODY frame ────────────────────────────── #
    def _dynamic_window(self, speed_cap: float) -> dict:
        dt = 0.10

        cur_vf, cur_vl = self._world_to_body(
            self.vel_world[0],
            self.vel_world[1],
            self.commanded_yaw
        )

        vf_min = max(0.0, cur_vf - self.A_MAX * dt)
        vf_max = min(speed_cap, cur_vf + self.A_MAX * dt)

        vl_min = max(-self.VY_MAX, cur_vl - self.A_MAX * dt)
        vl_max = min(self.VY_MAX, cur_vl + self.A_MAX * dt)

        # If speed_cap is zero, all forward samples become zero.
        return {
            'vf': np.linspace(vf_min, vf_max, self.V_SMP),
            'vl': np.linspace(vl_min, vl_max, self.VY_SMP),
        }

    # ─── Vectorised sample + scoring ─────────────────────────────── #
    def _best_sample(self, dw: dict):
        VF, VL = np.meshgrid(dw['vf'], dw['vl'], indexing='ij')
        VF_flat = VF.ravel()
        VL_flat = VL.ravel()

        c = math.cos(self.commanded_yaw)
        s = math.sin(self.commanded_yaw)

        VX = c * VF_flat - s * VL_flat
        VY = s * VF_flat + c * VL_flat

        dt = self.SIM_T / self.SIM_S
        steps = np.arange(1, self.SIM_S + 1, dtype=float)

        tx = self.pos[0] + VX[:, None] * dt * steps
        ty = self.pos[1] + VY[:, None] * dt * steps

        # ── Clearance via precomputed EDT ─────────────────────────── #
        if self._dist_transform is not None and self.grid is not None:
            res = self.grid.info.resolution
            ox = self.grid.info.origin.position.x
            oy = self.grid.info.origin.position.y
            W = self.grid.info.width
            H = self.grid.info.height

            gx = np.floor((tx - ox) / res).astype(int)
            gy = np.floor((ty - oy) / res).astype(int)

            in_bounds = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)

            clr = self._dist_transform[
                np.clip(gy, 0, H - 1),
                np.clip(gx, 0, W - 1)
            ]

            # Out of bounds is treated as unknown but not instantly fatal.
            # The local grid is centered on the vehicle, so short rollouts
            # should normally remain inside.
            clr = np.where(in_bounds, clr, 5.0)

            min_clr = clr.min(axis=1)
            mean_clr = clr[:, ::4].mean(axis=1)
        else:
            n = len(VF_flat)
            min_clr = np.full(n, 5.0)
            mean_clr = np.full(n, 5.0)

        valid = min_clr > self.HARD_STOP

        if not np.any(valid):
            return None

        # ── Scoring terms ─────────────────────────────────────────── #
        d0 = math.hypot(
            self.pos[0] - self.goal[0],
            self.pos[1] - self.goal[1]
        )

        d1 = np.hypot(
            tx[:, -1] - self.goal[0],
            ty[:, -1] - self.goal[1]
        )

        goal_score = (d0 - d1) / (self.SIM_T + 1e-6)

        spd_score = VF_flat / (self.V_MAX + 1e-6)
        lat_penalty = np.abs(VL_flat) / (self.VY_MAX + 1e-6)

        vel_norm = np.hypot(VX, VY)
        goal_dx = self.goal[0] - self.pos[0]
        goal_dy = self.goal[1] - self.pos[1]
        goal_norm = math.hypot(goal_dx, goal_dy) + 1e-6

        with np.errstate(invalid='ignore', divide='ignore'):
            heading_score = np.where(
                vel_norm > 1e-6,
                (VX * goal_dx + VY * goal_dy) / (vel_norm * goal_norm),
                0.0
            )

        smt_score = -(
            np.abs(VF_flat - self.last_cmd[0])
            + np.abs(VL_flat - self.last_cmd[1])
        )

        total = (
            self.W_G * goal_score
            + self.W_C * mean_clr
            + self.W_S * spd_score
            + self.W_H * heading_score
            + self.W_SM * smt_score
            - self.W_LAT * lat_penalty
        )

        total[~valid] = -np.inf

        idx = int(np.argmax(total))

        if not np.isfinite(total[idx]):
            return None

        return float(VF_flat[idx]), float(VL_flat[idx])

    # ─── Progress / stuck logic ──────────────────────────────────── #
    def _update_stuck_state(self, dist: float) -> bool:
        now = self._now_sec()

        if self.last_progress_check_time is None:
            self.last_progress_check_time = now
            self.last_progress_dist = dist
            return False

        dt = now - self.last_progress_check_time

        if dt < self.STUCK_T:
            return False

        progress = self.last_progress_dist - dist

        self.last_progress_check_time = now
        self.last_progress_dist = dist

        return progress < self.MIN_PROGRESS

    # ─── Grid queries ────────────────────────────────────────────── #
    def _grid_cell(self, wx: float, wy: float):
        if self.grid is None:
            return None

        res = self.grid.info.resolution
        ox = self.grid.info.origin.position.x
        oy = self.grid.info.origin.position.y
        W = self.grid.info.width
        H = self.grid.info.height

        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)

        if 0 <= gx < W and 0 <= gy < H:
            return gx, gy

        return None

    def _is_occupied(self, wx: float, wy: float) -> bool:
        cell = self._grid_cell(wx, wy)

        if cell is None:
            return False

        gx, gy = cell
        W = self.grid.info.width

        return self.grid.data[gy * W + gx] > 50

    def _clearance_at(self, wx: float, wy: float) -> float:
        if self._dist_transform is None:
            return 5.0

        cell = self._grid_cell(wx, wy)

        if cell is None:
            return 5.0

        gx, gy = cell

        return float(self._dist_transform[gy, gx])

    def _directional_clearance(self, yaw_dir: float, max_dist: float) -> float:
        d = 0.0

        while d <= max_dist:
            x = self.pos[0] + d * math.cos(yaw_dir)
            y = self.pos[1] + d * math.sin(yaw_dir)

            if self._is_occupied(x, y):
                return d

            d += self.REC_STEP

        return max_dist

    # ─── Publishers ──────────────────────────────────────────────── #
    def _publish_vel_world(self, vx: float, vy: float, yaw_cmd: float | None = None):
        sp = TrajectorySetpoint()

        sp.timestamp = self.get_clock().now().nanoseconds // 1000

        # Velocity control in XY, altitude hold in Z.
        sp.position = [float('nan'), float('nan'), -self.ALT]
        sp.velocity = [float(vx), float(vy), 0.0]
        sp.acceleration = [float('nan'), float('nan'), float('nan')]
        sp.jerk = [float('nan'), float('nan'), float('nan')]

        sp.yaw = float(yaw_cmd) if yaw_cmd is not None else float('nan')
        sp.yawspeed = float('nan')

        self.cmd_pub.publish(sp)

    def _pub_status(self, s: str):
        msg = String()
        msg.data = s
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()