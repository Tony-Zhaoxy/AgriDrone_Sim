"""
orb_slam3_bridge.py
===================
Bridges ORB_SLAM3 pose estimates to PX4's EKF2 via uXRCE-DDS.

Drop-in replacement for vio_bridge.py (OpenVINS).  The two nodes publish to
the same /fmu/in/vehicle_visual_odometry topic, so you run ONE of them —
never both at the same time.

Why ORB_SLAM3 over OpenVINS in a greenhouse?
  - Loop-closure: ORB_SLAM3 can recognise revisited row-end posts and
    correct accumulated drift, which OpenVINS MSCKF cannot.
  - Relocalization: after tracking loss (e.g. a sudden turn), ORB_SLAM3
    can recover using the stored map; OpenVINS must be restarted.
  - Stereo-inertial mode: fuses both IR cameras + D455 IMU, eliminating
    scale ambiguity (same benefit as stereo OpenVINS, but with the bonus of
    map-based drift correction).

Coordinate convention:
  ORB_SLAM3 world frame (as published by community ROS 2 wrappers such as
  suchetanrs/ORB-SLAM3-ROS2-Docker and Mechazo11/ros2_orb_slam3):
    x = forward (camera optical axis projected to horizontal)
    y = left
    z = up
  This matches ROS REP-103 / ENU, so the same ENU→NED conversion used in
  vio_bridge.py applies directly.

  If your wrapper publishes in camera-optical convention (x=right, y=down,
  z=forward), set the `coord_convention` parameter to 'optical'.  The bridge
  will rotate to ENU before forwarding to PX4.

Tracking loss handling:
  When ORB_SLAM3 loses tracking the pose topic goes silent.  A watchdog
  timer detects this and stops forwarding odometry to PX4 (so EKF2 coasts
  on IMU only rather than receiving stale data).

Relocalization jump handling:
  ORB_SLAM3 may emit a discontinuous pose jump after recovering from a lost
  track or after loop-closure.  The bridge detects jumps larger than
  `max_jump_m` and resets the velocity estimator window to avoid publishing
  a physically-impossible velocity spike to EKF2.

Topic map:
  SUB  /orb_slam3/camera_pose          (geometry_msgs/PoseStamped)
  SUB  /orb_slam3/tracking_image       (sensor_msgs/Image) [optional]
  PUB  /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry)
  PUB  /orb_slam3/status               (std_msgs/String)
"""

import math
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import String


class OrbSlam3Bridge(Node):
    """Converts ORB_SLAM3 camera pose to PX4 VehicleOdometry (NED)."""

    def __init__(self):
        super().__init__('orb_slam3_bridge')

        # ── Parameters ───────────────────────────────────────────────── #
        self.declare_parameter('pose_topic',        '/orb_slam3/camera_pose')
        self.declare_parameter('coord_convention',  'ros')   # 'ros' | 'optical'
        # Fixed covariance values (no covariance from ORB_SLAM3 pose topic)
        self.declare_parameter('pos_variance',      0.02)    # m²  (~14 cm 1-σ)
        self.declare_parameter('rot_variance',      0.01)    # rad²
        # Tracking watchdog: how long to wait before declaring tracking lost
        self.declare_parameter('tracking_timeout_sec', 0.50)
        # Relocalization / loop-closure jump detector
        self.declare_parameter('max_jump_m',        0.50)    # m

        self.POSE_TOPIC   = str(self.get_parameter('pose_topic').value)
        self.CONVENTION   = str(self.get_parameter('coord_convention').value)
        self.POS_VAR      = float(self.get_parameter('pos_variance').value)
        self.ROT_VAR      = float(self.get_parameter('rot_variance').value)
        self.TRACK_TIMEOUT = float(self.get_parameter('tracking_timeout_sec').value)
        self.MAX_JUMP     = float(self.get_parameter('max_jump_m').value)

        if self.CONVENTION not in ('ros', 'optical'):
            self.get_logger().error(
                f"Unknown coord_convention='{self.CONVENTION}'. Using 'ros'."
            )
            self.CONVENTION = 'ros'

        # ── QoS ─────────────────────────────────────────────────────── #
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ── Subscriptions ────────────────────────────────────────────── #
        self.create_subscription(
            PoseStamped,
            self.POSE_TOPIC,
            self.pose_cb,
            10,
        )

        # ── Publishers ───────────────────────────────────────────────── #
        self.odom_pub   = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', px4_qos)
        self.status_pub = self.create_publisher(String, '/orb_slam3/status', 10)

        # ── State ────────────────────────────────────────────────────── #
        self.origin_enu:  np.ndarray | None = None
        self.last_ned:    np.ndarray | None = None   # for jump detection
        self.got_first = False
        self.msg_count = 0

        # Sliding window for finite-difference velocity estimation.
        # Each entry: (timestamp_us: int, ned_pos: np.ndarray)
        self._pose_history: deque = deque(maxlen=5)

        # Tracking watchdog
        self._last_pose_time = 0.0
        self._tracking_ok    = False
        self.create_timer(0.2, self._watchdog_cb)  # 5 Hz check

        self.get_logger().info(
            f'ORB-SLAM3 Bridge started | '
            f'topic={self.POSE_TOPIC} | convention={self.CONVENTION} | '
            f'pos_var={self.POS_VAR} rot_var={self.ROT_VAR}'
        )

    # ──────────────────────────────────────────────────────────────────── #
    # Watchdog
    # ──────────────────────────────────────────────────────────────────── #
    def _watchdog_cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._tracking_ok and (now - self._last_pose_time) > self.TRACK_TIMEOUT:
            self._tracking_ok = False
            self._pose_history.clear()   # reset velocity estimator
            self.get_logger().warn(
                'ORB_SLAM3 tracking LOST (pose topic silent). '
                'PX4 EKF2 will coast on IMU.',
                throttle_duration_sec=1.0,
            )
            self._pub_status('TRACKING_LOST')

    # ──────────────────────────────────────────────────────────────────── #
    # Math helpers (shared with vio_bridge.py)
    # ──────────────────────────────────────────────────────────────────── #
    @staticmethod
    def _normalize_quat_xyzw(q: np.ndarray) -> np.ndarray | None:
        n = np.linalg.norm(q)
        if not np.isfinite(n) or n < 1e-12:
            return None
        return q / n

    @staticmethod
    def _quat_xyzw_to_rotmat(q: np.ndarray) -> np.ndarray:
        x, y, z, w = q
        xx, yy, zz = x*x, y*y, z*z
        return np.array([
            [1 - 2*(yy+zz),   2*(x*y - w*z),   2*(x*z + w*y)],
            [2*(x*y + w*z),   1 - 2*(xx+zz),   2*(y*z - w*x)],
            [2*(x*z - w*y),   2*(y*z + w*x),   1 - 2*(xx+yy)],
        ], dtype=np.float64)

    @staticmethod
    def _rotmat_to_quat_wxyz(R: np.ndarray) -> list:
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0.0:
            S = math.sqrt(tr + 1.0) * 2.0
            w = 0.25 * S
            x = (R[2,1] - R[1,2]) / S
            y = (R[0,2] - R[2,0]) / S
            z = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
        q = np.array([w, x, y, z], dtype=np.float64)
        n = np.linalg.norm(q)
        if not np.isfinite(n) or n < 1e-12:
            return [1.0, 0.0, 0.0, 0.0]
        return (q / n).tolist()

    # Rotation from ORB_SLAM3 optical frame to ENU:
    #   optical: x=right, y=down, z=forward
    #   ENU:     x=east(fwd), y=north(left), z=up
    # R_enu_optical maps: x_opt→z_enu? No — the mapping depends on drone heading.
    # For the generic rotation at SLAM init: x_enu = z_opt, y_enu = -x_opt, z_enu = -y_opt
    _R_OPT_TO_ENU = np.array([
        [ 0.0, -1.0,  0.0],
        [ 0.0,  0.0, -1.0],
        [ 1.0,  0.0,  0.0],
    ], dtype=np.float64)

    @staticmethod
    def _enu_to_ned_pos(p_enu: np.ndarray) -> np.ndarray:
        """ENU position → NED position: [N,E,D] = [y, x, -z]"""
        return np.array([p_enu[1], p_enu[0], -p_enu[2]], dtype=np.float64)

    @staticmethod
    def _enu_rot_to_ned_rot(R_enu: np.ndarray) -> np.ndarray:
        T = np.array([
            [0.0, 1.0,  0.0],
            [1.0, 0.0,  0.0],
            [0.0, 0.0, -1.0],
        ], dtype=np.float64)
        return T @ R_enu

    # ──────────────────────────────────────────────────────────────────── #
    # Pose callback
    # ──────────────────────────────────────────────────────────────────── #
    def pose_cb(self, msg: PoseStamped):
        now_ns = self.get_clock().now().nanoseconds
        self._last_pose_time = now_ns * 1e-9

        pos = msg.pose.position
        ori = msg.pose.orientation

        pos_raw = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
        if not np.all(np.isfinite(pos_raw)):
            self.get_logger().warn(
                'Non-finite ORB_SLAM3 position — skipping.',
                throttle_duration_sec=1.0,
            )
            return

        q_xyzw = self._normalize_quat_xyzw(
            np.array([ori.x, ori.y, ori.z, ori.w], dtype=np.float64)
        )
        if q_xyzw is None:
            self.get_logger().warn(
                'Invalid ORB_SLAM3 quaternion — skipping.',
                throttle_duration_sec=1.0,
            )
            return

        # ── Coordinate convention conversion ─────────────────────────── #
        if self.CONVENTION == 'optical':
            # Rotate position from optical frame to ENU
            pos_enu = self._R_OPT_TO_ENU @ pos_raw
            # Rotate orientation: R_enu_world = R_opt_to_enu @ R_opt_world
            R_opt = self._quat_xyzw_to_rotmat(q_xyzw)
            R_enu = self._R_OPT_TO_ENU @ R_opt
        else:
            # 'ros' convention: already ENU-compatible
            pos_enu = pos_raw
            R_enu   = self._quat_xyzw_to_rotmat(q_xyzw)

        # ── Origin lock on first valid pose ──────────────────────────── #
        if not self.got_first:
            self.origin_enu = pos_enu.copy()
            self.got_first  = True
            self._tracking_ok = True
            self.get_logger().info(
                f'ORB_SLAM3 origin locked at ENU '
                f'({pos_enu[0]:.3f}, {pos_enu[1]:.3f}, {pos_enu[2]:.3f})'
            )
            self._pub_status('TRACKING_OK')

        rel_enu = pos_enu - self.origin_enu

        # ── Relocalization / loop-closure jump detection ──────────────── #
        ned_pos = self._enu_to_ned_pos(rel_enu)
        if self.last_ned is not None:
            jump = float(np.linalg.norm(ned_pos - self.last_ned))
            if jump > self.MAX_JUMP:
                self.get_logger().warn(
                    f'ORB_SLAM3 position jump detected: {jump:.3f} m '
                    f'(relocalization or loop closure). '
                    f'Resetting velocity estimator.',
                )
                self._pose_history.clear()
                self._pub_status(f'JUMP_{jump:.2f}m')

        self.last_ned     = ned_pos.copy()
        self._tracking_ok = True

        # ── NED attitude ─────────────────────────────────────────────── #
        R_ned    = self._enu_rot_to_ned_rot(R_enu)
        q_ned_wxyz = self._rotmat_to_quat_wxyz(R_ned)

        # ── Build VehicleOdometry ─────────────────────────────────────── #
        odom = VehicleOdometry()
        odom.timestamp = now_ns // 1000   # µs

        sample_ns = (int(msg.header.stamp.sec) * 1_000_000_000
                     + int(msg.header.stamp.nanosec))
        odom.timestamp_sample = (sample_ns // 1000) if sample_ns > 0 else odom.timestamp

        odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        odom.position   = ned_pos.tolist()
        odom.q          = q_ned_wxyz

        # Fixed covariance (ORB_SLAM3 PoseStamped carries no covariance)
        odom.position_variance    = [self.POS_VAR] * 3
        odom.orientation_variance = [self.ROT_VAR] * 3

        # ── Finite-difference velocity estimation ─────────────────────── #
        self._pose_history.append((odom.timestamp, ned_pos.copy()))

        if len(self._pose_history) >= 2:
            t0, p0 = self._pose_history[0]
            t1, p1 = self._pose_history[-1]
            dt_sec = (t1 - t0) * 1e-6   # µs → s
            if dt_sec >= 0.05:           # require ≥50 ms window for good SNR
                vel_ned = (p1 - p0) / dt_sec
                odom.velocity       = vel_ned.tolist()
                odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            else:
                odom.velocity       = [float('nan')] * 3
                odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        else:
            odom.velocity       = [float('nan')] * 3
            odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN

        self.odom_pub.publish(odom)

        self.msg_count += 1
        if self.msg_count % 100 == 0:
            self.get_logger().info(
                f'ORB_SLAM3 NED pos: ({ned_pos[0]:.2f}, '
                f'{ned_pos[1]:.2f}, {ned_pos[2]:.2f})',
                throttle_duration_sec=2.0,
            )

    # ──────────────────────────────────────────────────────────────────── #
    # Helpers
    # ──────────────────────────────────────────────────────────────────── #
    def _pub_status(self, s: str):
        msg = String()
        msg.data = s
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OrbSlam3Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
