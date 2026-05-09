"""
mission_executor.py
===================
Marker-aware multi-row mission controller with a strict pre-OFFBOARD safety gate.

Important safety logic:
  This node does NOT request OFFBOARD immediately after startup.

  New mission start sequence:
    1. Wait for PX4 /fmu/out/vehicle_local_position
    2. Publish OFFBOARD heartbeat and hold setpoints for warm-up
    3. Wait for fresh and stable VIO input on /fmu/in/vehicle_visual_odometry
    4. Wait for MARKER_INIT if require_marker_init=True
    5. Request OFFBOARD mode
    6. Wait for PX4 OFFBOARD confirmation, or fallback after offboard_wait_sec
    7. Start the forward mission

Phase flow:
  IDLE -> WAIT_VIO_READY -> WAIT_MARKER_INIT -> WAIT_OFFBOARD -> FORWARD
             -> [MARKER_UTURN_LEFT / RIGHT] -> UTURN_ROT1 -> UTURN_MOVE -> UTURN_ROT2 -> FORWARD
             -> [MARKER_LAND]               -> LANDING -> DONE

Marker events, received as JSON on /marker/event:
  MARKER_INIT        — Start gate marker detected. This only records permission
                       to start; it does NOT directly start the mission.
  MARKER_UTURN_LEFT  — End of row; U-turn by rotating LEFT, moving to the next row,
                       then rotating again before resuming forward flight.
  MARKER_UTURN_RIGHT — Same as above, but rotating RIGHT.
  MARKER_LAND        — Final marker reached; stop and land.

U-turn geometry:
  ROT1   : yaw ±90° according to marker direction
  MOVE   : translate row_spacing_m laterally in the turned direction
  ROT2   : yaw another ±90° so the drone faces back along the row
  FORWARD: send new row goal, mission_distance m in the new heading

Behavior:
  - Assumes vehicle is already airborne when node starts.
  - Does NOT arm or take off.
  - Requests OFFBOARD only after VIO/local-position/marker gates are satisfied.
  - Publishes continuous OFFBOARD heartbeat at 10 Hz.
  - Delegates forward motion to DWA planner via /mission/goal.
  - Uses direct position setpoints during waiting, holding, and U-turn phases.
"""

import json
import math
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
    OffboardControlMode,
    VehicleCommand,
)
from std_msgs.msg import String


class Phase(Enum):
    IDLE             = auto()  # heartbeat warm-up while holding current pose
    WAIT_VIO_READY   = auto()  # wait for fresh PX4 local position + VIO input
    WAIT_MARKER_INIT = auto()  # wait until MARKER_INIT event before OFFBOARD
    WAIT_OFFBOARD    = auto()  # OFFBOARD requested, waiting for PX4 confirmation
    FORWARD          = auto()  # fly forward via DWA planner
    UTURN_ROT1       = auto()  # rotate 90° toward next row
    UTURN_MOVE       = auto()  # translate row_spacing_m
    UTURN_ROT2       = auto()  # rotate 90° more, now facing back
    LANDING          = auto()  # send PX4 land command
    DONE             = auto()  # hold final pose


class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')

        # ── Mission parameters ───────────────────────────────────────── #
        self.declare_parameter('takeoff_height',   1.50)
        self.declare_parameter('mission_distance', 10.0)
        self.declare_parameter('position_tol',      0.30)
        self.declare_parameter('yaw_tol_deg',        8.0)
        self.declare_parameter('hover_settle_sec',   2.0)
        self.declare_parameter('offboard_wait_sec',  1.0)
        self.declare_parameter('uturn_settle_sec',   2.0)
        self.declare_parameter('row_spacing_m',      2.0)
        self.declare_parameter('uturn_speed',        0.25)
        self.declare_parameter('planner_goal_topic', '/mission/goal')

        # Pre-OFFBOARD safety gate parameters
        self.declare_parameter('require_marker_init', True)
        self.declare_parameter('min_vio_samples', 30)
        self.declare_parameter('vio_timeout_sec', 0.50)
        self.declare_parameter('vio_stable_sec', 2.00)
        self.declare_parameter('local_pos_timeout_sec', 0.50)
        self.declare_parameter('min_local_pos_samples', 10)

        self.ALT           = float(self.get_parameter('takeoff_height').value)
        self.DIST          = float(self.get_parameter('mission_distance').value)
        self.POS_TOL       = float(self.get_parameter('position_tol').value)
        self.YAW_TOL       = math.radians(float(self.get_parameter('yaw_tol_deg').value))
        self.HOV_SETT      = float(self.get_parameter('hover_settle_sec').value)
        self.OFFBOARD_WAIT = float(self.get_parameter('offboard_wait_sec').value)
        self.UTURN_SETT    = float(self.get_parameter('uturn_settle_sec').value)
        self.ROW_SPACING   = float(self.get_parameter('row_spacing_m').value)
        self.UTURN_SPD     = float(self.get_parameter('uturn_speed').value)
        self.GOAL_TOPIC    = str(self.get_parameter('planner_goal_topic').value)
        self.REQUIRE_INIT  = bool(self.get_parameter('require_marker_init').value)

        self.MIN_VIO_SAMPLES       = int(self.get_parameter('min_vio_samples').value)
        self.VIO_TIMEOUT_SEC       = float(self.get_parameter('vio_timeout_sec').value)
        self.VIO_STABLE_SEC        = float(self.get_parameter('vio_stable_sec').value)
        self.LOCAL_POS_TIMEOUT_SEC = float(self.get_parameter('local_pos_timeout_sec').value)
        self.MIN_LOCAL_POS_SAMPLES = int(self.get_parameter('min_local_pos_samples').value)

        # ── QoS ─────────────────────────────────────────────────────── #
        px4_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        px4_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ───────────────────────────────────────────────── #
        self.ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_pub_qos)

        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_pub_qos)

        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_pub_qos)

        self.goal_pub = self.create_publisher(
            TrajectorySetpoint, self.GOAL_TOPIC, 10)

        self.status_pub = self.create_publisher(
            String, '/mission/status', 10)

        # ── Subscriptions ────────────────────────────────────────────── #
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.pos_cb,
            px4_sub_qos,
        )

        self.create_subscription(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            self.vio_cb,
            px4_sub_qos,
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_cb,
            px4_sub_qos,
        )

        self.create_subscription(
            String,
            '/marker/event',
            self.marker_cb,
            10,
        )

        # ── State ────────────────────────────────────────────────────── #
        self.pos = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.have_pos = False

        self.local_pos_count = 0
        self.last_local_pos_time = None

        self.vio_msg_count = 0
        self.last_vio_time = None
        self.vio_ready_since = None

        self.marker_init_received = False

        self.phase = Phase.IDLE
        self.phase_t = time.time()

        self.start_xy = [0.0, 0.0]
        self.goal_xy = [0.0, 0.0]
        self.hold_yaw = 0.0

        # Row heading tracks the direction of the current row.
        # It flips by π after each U-turn.
        self.row_heading = 0.0
        self.row_count = 0

        # U-turn state
        self.uturn_dir = 'left'
        self.uturn_target_yaw1 = 0.0
        self.uturn_target_yaw2 = 0.0
        self.uturn_start_pos = [0.0, 0.0]
        self.uturn_target_pos = [0.0, 0.0]

        # OFFBOARD handshake
        self.offboard_requested = False
        self.offboard_request_time = 0.0
        self.offboard_active = False

        # Marker event queue
        self._marker_queue: list = []

        self.loop_dt = 0.10  # 10 Hz
        self.create_timer(self.loop_dt, self.loop)

        self.get_logger().info(
            f'MissionExecutor ready | alt={self.ALT}m dist={self.DIST}m '
            f'row_spacing={self.ROW_SPACING}m require_init={self.REQUIRE_INIT} '
            f'vio_samples={self.MIN_VIO_SAMPLES} vio_stable={self.VIO_STABLE_SEC}s'
        )

    # ──────────────────────────────────────────────────────────────────── #
    # Callbacks
    # ──────────────────────────────────────────────────────────────────── #
    def pos_cb(self, msg: VehicleLocalPosition):
        self.pos = [msg.x, msg.y, msg.z]
        self.yaw = msg.heading
        self.have_pos = True
        self.local_pos_count += 1
        self.last_local_pos_time = self.get_clock().now()

    def vio_cb(self, msg: VehicleOdometry):
        # vio_bridge should publish this only after it has a valid VIO pose.
        # We still require multiple fresh samples before accepting it as ready.
        self.vio_msg_count += 1
        self.last_vio_time = self.get_clock().now()

        if self.vio_msg_count >= self.MIN_VIO_SAMPLES and self.vio_ready_since is None:
            self.vio_ready_since = self.get_clock().now()

    def status_cb(self, msg: VehicleStatus):
        self.offboard_active = (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def marker_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._marker_queue.append(data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Bad marker JSON: {msg.data}')

    # ──────────────────────────────────────────────────────────────────── #
    # Readiness helpers
    # ──────────────────────────────────────────────────────────────────── #
    def _age_sec(self, stamp):
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _local_position_ready(self) -> bool:
        if not self.have_pos:
            return False

        if self.local_pos_count < self.MIN_LOCAL_POS_SAMPLES:
            return False

        if self._age_sec(self.last_local_pos_time) > self.LOCAL_POS_TIMEOUT_SEC:
            return False

        return (
            all(math.isfinite(v) for v in self.pos)
            and math.isfinite(self.yaw)
        )

    def _vio_ready(self) -> bool:
        if self.vio_msg_count < self.MIN_VIO_SAMPLES:
            return False

        if self._age_sec(self.last_vio_time) > self.VIO_TIMEOUT_SEC:
            self.vio_ready_since = None
            return False

        if self.vio_ready_since is None:
            return False

        stable_time = (self.get_clock().now() - self.vio_ready_since).nanoseconds * 1e-9
        return stable_time >= self.VIO_STABLE_SEC

    def _preoffboard_ready(self) -> bool:
        return self._local_position_ready() and self._vio_ready()

    # ──────────────────────────────────────────────────────────────────── #
    # Motion / PX4 helpers
    # ──────────────────────────────────────────────────────────────────── #
    def _dist_to(self, x: float, y: float) -> float:
        return math.hypot(self.pos[0] - x, self.pos[1] - y)

    def _angle_diff(self, a: float, b: float) -> float:
        """Signed difference a - b, wrapped to [-π, π]."""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def _send_cmd(self, cmd: int, p1: float = 0.0, p2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = int(cmd)
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def _heartbeat(self, use_position: bool = True):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = bool(use_position)
        msg.velocity = bool(not use_position)
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.ocm_pub.publish(msg)

    def _pos_setpoint(self, x: float, y: float, z_ned: float, yaw: float):
        sp = TrajectorySetpoint()
        sp.timestamp = self.get_clock().now().nanoseconds // 1000
        sp.position = [float(x), float(y), float(z_ned)]
        sp.velocity = [float('nan')] * 3
        sp.acceleration = [float('nan')] * 3
        sp.jerk = [float('nan')] * 3
        sp.yaw = float(yaw)
        sp.yawspeed = float('nan')
        self.sp_pub.publish(sp)

    def _request_offboard(self):
        if self.offboard_requested:
            return

        self.get_logger().info('Requesting OFFBOARD mode...')
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        self.offboard_requested = True
        self.offboard_request_time = time.time()

        self.phase = Phase.WAIT_OFFBOARD
        self.phase_t = time.time()

    def _send_goal(self, x: float, y: float):
        """Send forward goal to DWA planner."""
        sp = TrajectorySetpoint()
        sp.timestamp = self.get_clock().now().nanoseconds // 1000
        sp.position = [float(x), float(y), -float(self.ALT)]
        sp.velocity = [float('nan')] * 3
        sp.acceleration = [float('nan')] * 3
        sp.jerk = [float('nan')] * 3
        sp.yaw = float('nan')
        sp.yawspeed = float('nan')
        self.goal_pub.publish(sp)

    def _pub_status(self, s: str):
        msg = String()
        msg.data = f'[{self.phase.name}] {s}'
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data, throttle_duration_sec=1.0)

    def _start_forward(self):
        """Set goal in current row_heading direction and switch to FORWARD."""
        self.start_xy = [self.pos[0], self.pos[1]]
        self.goal_xy = [
            self.pos[0] + self.DIST * math.cos(self.row_heading),
            self.pos[1] + self.DIST * math.sin(self.row_heading),
        ]
        self.row_count += 1

        self.get_logger().info(
            f'Row {self.row_count} start: pos=({self.pos[0]:.2f},{self.pos[1]:.2f}) '
            f'heading={math.degrees(self.row_heading):.1f}° '
            f'goal=({self.goal_xy[0]:.2f},{self.goal_xy[1]:.2f})'
        )

        self.phase = Phase.FORWARD
        self.phase_t = time.time()

    def _start_uturn(self, direction: str):
        """Prepare U-turn state and switch to UTURN_ROT1."""
        self.uturn_dir = direction
        sign = -1.0 if direction == 'left' else 1.0  # left=CCW=negative in NED

        self.uturn_target_yaw1 = self.row_heading + sign * math.pi / 2.0
        self.uturn_target_yaw2 = self.row_heading + sign * math.pi

        lat_angle = self.uturn_target_yaw1
        self.uturn_start_pos = [self.pos[0], self.pos[1]]
        self.uturn_target_pos = [
            self.pos[0] + self.ROW_SPACING * math.cos(lat_angle),
            self.pos[1] + self.ROW_SPACING * math.sin(lat_angle),
        ]

        self.get_logger().info(
            f'U-turn {direction}: ROT1 yaw={math.degrees(self.uturn_target_yaw1):.1f}° '
            f'ROT2 yaw={math.degrees(self.uturn_target_yaw2):.1f}° '
            f'move to ({self.uturn_target_pos[0]:.2f},{self.uturn_target_pos[1]:.2f})'
        )

        self.phase = Phase.UTURN_ROT1
        self.phase_t = time.time()

    def _process_marker_queue(self):
        """Consume queued marker events and react based on current phase."""
        while self._marker_queue:
            data = self._marker_queue.pop(0)
            event = data.get('event', '')
            dist = data.get('distance_m', float('nan'))

            self.get_logger().info(
                f'Processing marker: {event}  dist={dist}m  phase={self.phase.name}'
            )

            if event == 'MARKER_INIT':
                self.marker_init_received = True
                self.get_logger().info(
                    'MARKER_INIT received — start gate recorded. '
                    'Mission will start only after VIO_READY and OFFBOARD.'
                )

            elif event in ('MARKER_UTURN_LEFT', 'MARKER_UTURN_RIGHT'):
                if self.phase == Phase.FORWARD:
                    direction = 'left' if event == 'MARKER_UTURN_LEFT' else 'right'
                    self.get_logger().info(
                        f'{event} received — starting U-turn {direction}.'
                    )
                    self._start_uturn(direction)

            elif event == 'MARKER_LAND':
                if self.phase == Phase.FORWARD:
                    self.get_logger().info(
                        'MARKER_LAND received — initiating landing.'
                    )
                    self.phase = Phase.LANDING
                    self.phase_t = time.time()

    # ──────────────────────────────────────────────────────────────────── #
    # Main loop, 10 Hz
    # ──────────────────────────────────────────────────────────────────── #
    def loop(self):
        # Always publish OFFBOARD heartbeat.
        # PX4 requires a continuous stream before and during OFFBOARD.
        use_position = self.phase not in (Phase.FORWARD,)
        self._heartbeat(use_position=use_position)

        if not self.have_pos:
            self.get_logger().warn(
                'Waiting for /fmu/out/vehicle_local_position...',
                throttle_duration_sec=2.0,
            )
            return

        # Marker events are processed every loop.
        # MARKER_INIT only sets a flag; it does not directly start the mission.
        self._process_marker_queue()

        # ── IDLE: heartbeat warm-up ──────────────────────────────────── #
        if self.phase == Phase.IDLE:
            self._pos_setpoint(self.pos[0], self.pos[1], self.pos[2], self.yaw)

            elapsed = time.time() - self.phase_t
            if elapsed < self.HOV_SETT:
                self._pub_status('warming up heartbeat before readiness checks')
                return

            self.phase = Phase.WAIT_VIO_READY
            self.phase_t = time.time()
            self.get_logger().info(
                'Heartbeat warm-up complete — waiting for VIO_READY.'
            )
            return

        # ── WAIT_VIO_READY: hard gate before OFFBOARD ───────────────── #
        elif self.phase == Phase.WAIT_VIO_READY:
            self._pos_setpoint(self.pos[0], self.pos[1], self.pos[2], self.yaw)

            if not self._local_position_ready():
                self._pub_status(
                    f'waiting PX4 local position: count={self.local_pos_count} '
                    f'age={self._age_sec(self.last_local_pos_time):.2f}s'
                )
                return

            if not self._vio_ready():
                self._pub_status(
                    f'waiting VIO_READY: count={self.vio_msg_count} '
                    f'age={self._age_sec(self.last_vio_time):.2f}s'
                )
                return

            self.get_logger().info(
                'VIO_READY confirmed — local position and visual odometry are fresh.'
            )

            if self.REQUIRE_INIT:
                self.phase = Phase.WAIT_MARKER_INIT
                self.phase_t = time.time()
                self.get_logger().info(
                    'Waiting for MARKER_INIT before requesting OFFBOARD.'
                )
            else:
                self.get_logger().info(
                    'require_marker_init=False — requesting OFFBOARD without marker gate.'
                )
                self._request_offboard()

            return

        # ── WAIT_MARKER_INIT: marker gate before OFFBOARD ────────────── #
        elif self.phase == Phase.WAIT_MARKER_INIT:
            self._pos_setpoint(self.pos[0], self.pos[1], self.pos[2], self.yaw)

            if not self._preoffboard_ready():
                self.get_logger().warn(
                    'VIO/local position became stale before OFFBOARD — returning to WAIT_VIO_READY.'
                )
                self.phase = Phase.WAIT_VIO_READY
                self.phase_t = time.time()
                return

            if not self.marker_init_received:
                self._pub_status(
                    'holding — VIO_READY, awaiting MARKER_INIT before OFFBOARD'
                )
                return

            self.get_logger().info(
                'VIO_READY + MARKER_INIT confirmed — requesting OFFBOARD mode.'
            )
            self._request_offboard()
            return

        # ── WAIT_OFFBOARD: wait until PX4 confirms OFFBOARD ──────────── #
        elif self.phase == Phase.WAIT_OFFBOARD:
            self._pos_setpoint(self.pos[0], self.pos[1], self.pos[2], self.yaw)

            if not self._preoffboard_ready():
                self.get_logger().warn(
                    'VIO/local position became stale while waiting for OFFBOARD. '
                    'Holding current pose and waiting for recovery.'
                )
                return

            if self.offboard_active:
                self.get_logger().info(
                    'OFFBOARD active — starting mission now.'
                )
                self.hold_yaw = self.yaw
                self.row_heading = self.yaw
                self._start_forward()
                return

            # Fallback in case /fmu/out/vehicle_status is not bridged.
            if self.offboard_requested and (
                time.time() - self.offboard_request_time
            ) > self.OFFBOARD_WAIT:
                self.get_logger().warn(
                    'OFFBOARD status not confirmed from /fmu/out/vehicle_status; '
                    'continuing after offboard_wait_sec fallback.'
                )
                self.hold_yaw = self.yaw
                self.row_heading = self.yaw
                self._start_forward()
                return

            self._pub_status('waiting for PX4 OFFBOARD active')
            return

        # ── FORWARD ──────────────────────────────────────────────────── #
        elif self.phase == Phase.FORWARD:
            self._send_goal(*self.goal_xy)
            self._pub_status(
                f'row={self.row_count} dist={self._dist_to(*self.goal_xy):.2f}m'
            )

            # Geometric goal reached without seeing land marker.
            if self._dist_to(*self.goal_xy) < self.POS_TOL:
                self.get_logger().info(
                    f'Row {self.row_count} goal reached. Holding, no land marker.'
                )
                self.phase = Phase.DONE
                self.phase_t = time.time()

        # ── UTURN_ROT1: rotate 90° toward next row ───────────────────── #
        elif self.phase == Phase.UTURN_ROT1:
            self._pos_setpoint(
                self.pos[0], self.pos[1], -self.ALT,
                self.uturn_target_yaw1,
            )

            yaw_err = abs(self._angle_diff(self.yaw, self.uturn_target_yaw1))
            self._pub_status(f'UTURN_ROT1 yaw_err={math.degrees(yaw_err):.1f}°')

            if yaw_err < self.YAW_TOL:
                self.get_logger().info('UTURN_ROT1 complete — moving laterally.')
                self.phase = Phase.UTURN_MOVE
                self.phase_t = time.time()

        # ── UTURN_MOVE: translate to next row ────────────────────────── #
        elif self.phase == Phase.UTURN_MOVE:
            tx, ty = self.uturn_target_pos
            self._pos_setpoint(tx, ty, -self.ALT, self.uturn_target_yaw1)

            dist = self._dist_to(tx, ty)
            self._pub_status(f'UTURN_MOVE dist={dist:.2f}m')

            if dist < self.POS_TOL:
                self.get_logger().info(
                    'UTURN_MOVE complete — rotating to row heading.'
                )
                self.phase = Phase.UTURN_ROT2
                self.phase_t = time.time()

        # ── UTURN_ROT2: rotate 90° more, now facing back ─────────────── #
        elif self.phase == Phase.UTURN_ROT2:
            self._pos_setpoint(
                self.pos[0], self.pos[1], -self.ALT,
                self.uturn_target_yaw2,
            )

            yaw_err = abs(self._angle_diff(self.yaw, self.uturn_target_yaw2))
            self._pub_status(f'UTURN_ROT2 yaw_err={math.degrees(yaw_err):.1f}°')

            if yaw_err < self.YAW_TOL:
                self.get_logger().info(
                    'UTURN_ROT2 complete — starting next row.'
                )
                self.row_heading = self.uturn_target_yaw2
                self.hold_yaw = self.row_heading
                self._start_forward()

        # ── LANDING ──────────────────────────────────────────────────── #
        elif self.phase == Phase.LANDING:
            self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._pub_status('landing command sent')
            self.get_logger().info('Landing command issued. -> DONE')

            self.phase = Phase.DONE
            self.phase_t = time.time()

        # ── DONE ─────────────────────────────────────────────────────── #
        elif self.phase == Phase.DONE:
            self._pos_setpoint(
                self.pos[0], self.pos[1], self.pos[2], self.hold_yaw
            )
            self._pub_status('holding at final position')


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()