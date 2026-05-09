"""
vio_bridge.py
=============
Relays OpenVINS pose estimates to PX4's EKF2 via uXRCE-DDS.

Handles:
  - ROS ENU frame  ->  PX4 NED frame conversion
  - Origin locking on first received pose
  - Covariance forwarding so EKF2 can weight VIO confidence correctly
  - Safer quaternion conversion via rotation matrices

Topic map:
  SUB  /poseimu               (geometry_msgs/PoseWithCovarianceStamped)
  PUB  /fmu/in/vehicle_visual_odometry  (px4_msgs/VehicleOdometry)
"""

import math
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleOdometry


class VIOBridge(Node):
    def __init__(self):
        super().__init__('vio_bridge')

        # PX4 uXRCE-DDS topics are commonly BEST_EFFORT
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/poseimu',
            self.vio_callback,
            10
        )

        self.pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            px4_qos
        )

        self.origin = None
        self.got_first = False
        self.msg_count = 0

        # Sliding window for finite-difference velocity estimation.
        # Each entry: (timestamp_us: int, ned_pos: np.ndarray)
        # 5 samples at ~30 Hz → ~150 ms window → good SNR for slow indoor flight.
        self._pose_history: deque = deque(maxlen=5)

        self.get_logger().info(
            'VIO Bridge started — waiting for first OpenVINS pose...'
        )

    # ────────────────────────────────────────────────────────────── #
    # Math helpers
    # ────────────────────────────────────────────────────────────── #
    @staticmethod
    def _normalize_quat_xyzw(q):
        q = np.asarray(q, dtype=np.float64)
        n = np.linalg.norm(q)
        if not np.isfinite(n) or n < 1e-12:
            return None
        return q / n

    @staticmethod
    def _quat_xyzw_to_rotmat(q):
        """
        Input quaternion order: [x, y, z, w]
        Returns rotation matrix R such that:
            p_world = R * p_body
        """
        x, y, z, w = q

        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        R = np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
            [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)]
        ], dtype=np.float64)
        return R

    @staticmethod
    def _rotmat_to_quat_wxyz(R):
        """
        Convert rotation matrix to quaternion in PX4 order [w, x, y, z].
        """
        R = np.asarray(R, dtype=np.float64)
        tr = R[0, 0] + R[1, 1] + R[2, 2]

        if tr > 0.0:
            S = math.sqrt(tr + 1.0) * 2.0
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S

        q = np.array([w, x, y, z], dtype=np.float64)
        n = np.linalg.norm(q)
        if not np.isfinite(n) or n < 1e-12:
            return [1.0, 0.0, 0.0, 0.0]
        q /= n
        return q.tolist()

    @staticmethod
    def _enu_rot_to_ned_rot(R_enu):
        """
        Convert a rotation matrix expressed in ENU world frame
        into the equivalent rotation matrix in NED world frame.

        ENU -> NED for vectors:
            [N, E, D]^T = T * [E, N, U]^T
        with
            T = [[0,1,0],
                 [1,0,0],
                 [0,0,-1]]
        """
        T = np.array([
            [0.0, 1.0,  0.0],
            [1.0, 0.0,  0.0],
            [0.0, 0.0, -1.0]
        ], dtype=np.float64)

        return T @ R_enu

    @staticmethod
    def _reorder_position_variance_enu_to_ned(cov):
        """
        cov is 6x6 PoseWithCovariance diagonal in ENU order:
          [x, y, z, rot_x, rot_y, rot_z]
        NED position variance order should be:
          [N, E, D] = [ENU y, ENU x, ENU z]
        """
        return [
            float(cov[7]),   # y -> N
            float(cov[0]),   # x -> E
            float(cov[14])   # z -> D
        ]

    @staticmethod
    def _reorder_orientation_variance_enu_to_ned(cov):
        """
        Approximate orientation variance remap by axis permutation.
        For small-angle variance, use:
          roll_NED  <- pitch_ENU
          pitch_NED <- roll_ENU
          yaw_NED   <- yaw_ENU
        This is a practical approximation for EKF weighting.
        """
        return [
            float(cov[28]),  # rot_y -> roll_NED
            float(cov[21]),  # rot_x -> pitch_NED
            float(cov[35])   # rot_z -> yaw_NED
        ]

    # ────────────────────────────────────────────────────────────── #
    # Callback
    # ────────────────────────────────────────────────────────────── #
    def vio_callback(self, msg: PoseWithCovarianceStamped):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        cov = msg.pose.covariance

        if len(cov) != 36:
            self.get_logger().warn(
                f'Unexpected covariance length: {len(cov)}',
                throttle_duration_sec=1.0
            )
            return

        pos_enu = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
        if not np.all(np.isfinite(pos_enu)):
            self.get_logger().warn('Received non-finite VIO position.', throttle_duration_sec=1.0)
            return

        q_xyzw = self._normalize_quat_xyzw([ori.x, ori.y, ori.z, ori.w])
        if q_xyzw is None:
            self.get_logger().warn('Received invalid VIO quaternion.', throttle_duration_sec=1.0)
            return

        # Lock origin on first valid pose
        if not self.got_first:
            self.origin = pos_enu.copy()
            self.got_first = True
            self.get_logger().info(
                f'VIO origin locked at ENU ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})'
            )

        # Relative position to locked origin
        rel_enu = pos_enu - self.origin

        # ENU -> NED: [N, E, D] = [y, x, -z]
        ned_pos = np.array([rel_enu[1], rel_enu[0], -rel_enu[2]], dtype=np.float64)

        # Quaternion / attitude conversion
        R_enu = self._quat_xyzw_to_rotmat(q_xyzw)
        R_ned = self._enu_rot_to_ned_rot(R_enu)
        q_ned_wxyz = self._rotmat_to_quat_wxyz(R_ned)

        odom = VehicleOdometry()

        # Current publish time
        odom.timestamp = self.get_clock().now().nanoseconds // 1000

        # Use source message time if available
        sample_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if sample_ns > 0:
            odom.timestamp_sample = sample_ns // 1000
        else:
            odom.timestamp_sample = odom.timestamp

        odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        odom.position = ned_pos.tolist()
        odom.q = q_ned_wxyz

        odom.position_variance = self._reorder_position_variance_enu_to_ned(cov)
        odom.orientation_variance = self._reorder_orientation_variance_enu_to_ned(cov)

        # Finite-difference velocity over a sliding window of poses.
        # Using oldest→newest gives a low-pass filtered estimate that is
        # noisy enough to require generous variance but useful to EKF2.
        self._pose_history.append((odom.timestamp, ned_pos.copy()))

        if len(self._pose_history) >= 2:
            t0, p0 = self._pose_history[0]
            t1, p1 = self._pose_history[-1]
            dt_sec = (t1 - t0) * 1e-6   # microseconds → seconds
            if dt_sec >= 0.05:           # require at least 50 ms window
                vel_ned = (p1 - p0) / dt_sec
                odom.velocity = vel_ned.tolist()
                odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            else:
                odom.velocity = [float('nan'), float('nan'), float('nan')]
                odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        else:
            odom.velocity = [float('nan'), float('nan'), float('nan')]
            odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN

        self.pub.publish(odom)

        self.msg_count += 1
        if self.msg_count % 100 == 0:
            self.get_logger().info(
                f'VIO NED pos: ({ned_pos[0]:.2f}, {ned_pos[1]:.2f}, {ned_pos[2]:.2f})',
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = VIOBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()