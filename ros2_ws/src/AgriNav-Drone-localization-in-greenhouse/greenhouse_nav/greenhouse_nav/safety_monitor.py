"""
safety_monitor.py
=================
Independent watchdog node. Monitors VIO health, local-position health,
altitude limits, and floor proximity. Triggers an emergency land if any
limit is exceeded.

Runs independently of planner / mission nodes.

Safety conditions monitored:
  1. VIO age            — if no VIO message for > vio_timeout_sec, land
  2. Local pos age      — if no local position for > pos_timeout_sec, land
  3. Max altitude       — if drone climbs above max_altitude_m, land
  4. Min floor clearance — if rangefinder shows < min_clearance_m, land

Topic map:
  SUB  /fmu/in/vehicle_visual_odometry   (px4_msgs/VehicleOdometry)
  SUB  /fmu/out/vehicle_local_position (px4_msgs/VehicleLocalPosition)
  SUB  /d455/distance_sensor             (sensor_msgs/Range)
  PUB  /fmu/in/vehicle_command           (px4_msgs/VehicleCommand)
  PUB  /safety/status                    (std_msgs/String)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleCommand
from sensor_msgs.msg import Range
from std_msgs.msg import String


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        self.declare_parameter('vio_timeout_sec', 1.0)
        self.declare_parameter('pos_timeout_sec', 1.0)
        self.declare_parameter('max_altitude_m', 2.20)
        self.declare_parameter('min_clearance_m', 0.15)
        self.declare_parameter('airborne_alt_m', 0.30)
        self.declare_parameter('enable_range', True)

        self.VIO_TIMEOUT = float(self.get_parameter('vio_timeout_sec').value)
        self.POS_TIMEOUT = float(self.get_parameter('pos_timeout_sec').value)
        self.MAX_ALT = float(self.get_parameter('max_altitude_m').value)
        self.MIN_CLR = float(self.get_parameter('min_clearance_m').value)
        self.AIRBORNE_ALT = float(self.get_parameter('airborne_alt_m').value)
        self.USE_RANGE = bool(self.get_parameter('enable_range').value)

        now = time.monotonic()
        self.last_vio_t = now
        self.last_pos_t = now

        self.current_alt = 0.0
        self.is_airborne = False
        self.land_sent = False
        self.last_range = None

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

        self.create_subscription(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            self.vio_cb,
            px4_sub_qos
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.pos_cb,
            px4_sub_qos
        )

        if self.USE_RANGE:
            self.create_subscription(
                Range,
                '/d455/distance_sensor',
                self.range_cb,
                10
            )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_pub_qos
        )

        self.status_pub = self.create_publisher(String, '/safety/status', 10)

        self.create_timer(0.20, self.check)  # 5 Hz

        self.get_logger().info(
            f'Safety Monitor active — '
            f'vio_timeout={self.VIO_TIMEOUT}s  '
            f'pos_timeout={self.POS_TIMEOUT}s  '
            f'max_alt={self.MAX_ALT}m  '
            f'min_clr={self.MIN_CLR}m'
        )

    # ─── Callbacks ───────────────────────────────────────────────── #
    def vio_cb(self, msg: VehicleOdometry):
        self.last_vio_t = time.monotonic()

    def pos_cb(self, msg: VehicleLocalPosition):
        self.last_pos_t = time.monotonic()
        self.current_alt = -float(msg.z)   # NED -> positive up
        self.is_airborne = self.current_alt > self.AIRBORNE_ALT

    def range_cb(self, msg: Range):
        r = float(msg.range)

        if not math.isfinite(r):
            return

        if msg.min_range > 0.0 and r < msg.min_range:
            return
        if msg.max_range > 0.0 and r > msg.max_range:
            return

        self.last_range = r

        if self.is_airborne and r < self.MIN_CLR:
            self._trigger_land(
                f'RANGE too low: {r:.2f} m < {self.MIN_CLR:.2f} m'
            )

    # ─── Watchdog ────────────────────────────────────────────────── #
    def check(self):
        now = time.monotonic()

        if not self.is_airborne:
            if self.land_sent:
                self.get_logger().info('Vehicle appears landed; resetting safety latch.')
            self.land_sent = False
            self._pub_status('GROUND')
            return

        # 1. VIO freshness
        vio_age = now - self.last_vio_t
        if vio_age > self.VIO_TIMEOUT:
            self._trigger_land(
                f'VIO lost for {vio_age:.2f}s (limit={self.VIO_TIMEOUT:.2f}s)'
            )
            return

        # 2. Local position freshness
        pos_age = now - self.last_pos_t
        if pos_age > self.POS_TIMEOUT:
            self._trigger_land(
                f'LOCAL_POS lost for {pos_age:.2f}s (limit={self.POS_TIMEOUT:.2f}s)'
            )
            return

        # 3. Altitude ceiling
        if self.current_alt > self.MAX_ALT:
            self._trigger_land(
                f'ALT too high: {self.current_alt:.2f} m > {self.MAX_ALT:.2f} m'
            )
            return

        if self.last_range is not None:
            self._pub_status(
                f'OK alt={self.current_alt:.2f}m vio_age={vio_age:.2f}s '
                f'pos_age={pos_age:.2f}s range={self.last_range:.2f}m'
            )
        else:
            self._pub_status(
                f'OK alt={self.current_alt:.2f}m vio_age={vio_age:.2f}s '
                f'pos_age={pos_age:.2f}s'
            )

    # ─── Emergency land ──────────────────────────────────────────── #
    def _trigger_land(self, reason: str):
        self.get_logger().error(f'SAFETY TRIGGER: {reason}')
        self._pub_status(f'EMERGENCY_LAND: {reason}')

        if self.land_sent:
            return

        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        msg.param1 = 0.0
        msg.param2 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.cmd_pub.publish(msg)
        self.land_sent = True
        self.get_logger().warn('NAV_LAND command sent to PX4.')

    def _pub_status(self, s: str):
        msg = String()
        msg.data = s
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()