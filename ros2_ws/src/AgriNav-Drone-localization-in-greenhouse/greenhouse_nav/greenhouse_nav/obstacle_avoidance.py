"""
obstacle_avoidance.py
=====================
Publishes PX4 ObstacleDistance messages from the D455 depth stream.

This node feeds PX4's built-in Collision Prevention (CP) module, which
acts as a hardware-level safety net independent of the DWA planner.
Even if the DWA planner fails to react in time, CP can brake the drone.

Design in this version:
  - Use only the camera's forward horizontal FOV
  - Fill a continuous front sector in ObstacleDistance.distances[]
  - Use angle_offset so that the center of the covered bins points forward
  - Do NOT wrap bins around the 360 deg array
  - Keep all uncovered directions as 65535 (unknown)

Assumptions:
  - Camera optical axis points forward
  - depth image columns run from left FOV to right FOV
  - body frame is FRD, with positive angles rotating clockwise when viewed from above

Topic map:
  SUB  /d455/depth/image_rect_raw   (sensor_msgs/Image, uint16 mm)
  PUB  /fmu/in/obstacle_distance    (px4_msgs/ObstacleDistance)
"""

import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import ObstacleDistance
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class ObstacleAvoidanceCP(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_cp')

        # Camera / distance parameters
        self.declare_parameter('hfov_deg', 87.0)
        self.declare_parameter('min_dist_cm', 20)
        self.declare_parameter('max_dist_cm', 500)

        # Row band to sample (fraction of image height)
        self.declare_parameter('row_frac_lo', 0.25)
        self.declare_parameter('row_frac_hi', 0.75)

        # Conservative depth estimate: lower percentile = closer obstacle estimate
        self.declare_parameter('depth_percentile', 10)

        # Validity thresholds inside each angular bin
        self.declare_parameter('min_valid_points', 30)
        self.declare_parameter('min_valid_ratio', 0.02)

        # Topic name
        self.declare_parameter('depth_topic', '/d455/depth/image_rect_raw')

        self.HFOV = float(self.get_parameter('hfov_deg').value)
        self.D_MIN = int(self.get_parameter('min_dist_cm').value)
        self.D_MAX = int(self.get_parameter('max_dist_cm').value)

        self.ROW_LO = float(self.get_parameter('row_frac_lo').value)
        self.ROW_HI = float(self.get_parameter('row_frac_hi').value)

        self.PCNTL = float(self.get_parameter('depth_percentile').value)
        self.MIN_VALID_POINTS = int(self.get_parameter('min_valid_points').value)
        self.MIN_VALID_RATIO = float(self.get_parameter('min_valid_ratio').value)

        self.DEPTH_TOPIC = str(self.get_parameter('depth_topic').value)

        # PX4 ObstacleDistance uses 72 bins * 5 deg = 360 deg
        self.NUM_BINS = 72
        self.BIN_DEG = 360.0 / self.NUM_BINS  # 5.0 deg

        # Number of bins actually covered by the camera FOV
        self.NUM_COVER = max(1, int(round(self.HFOV / self.BIN_DEG)))

        # Angle offset for the first covered bin.
        # We fill distances[0:NUM_COVER] continuously from left->right across the camera FOV.
        # To center the covered sector on straight ahead (0 deg), set first-bin angle to:
        #   -0.5 * (covered_angle - bin_width)
        covered_angle = self.NUM_COVER * self.BIN_DEG
        self.ANGLE_OFFSET_DEG = -0.5 * (covered_angle - self.BIN_DEG)

        self.bridge = CvBridge()

        # For /fmu/in/*, RELIABLE is usually the safer choice
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            Image,
            self.DEPTH_TOPIC,
            self.depth_cb,
            10
        )

        self.pub = self.create_publisher(
            ObstacleDistance,
            '/fmu/in/obstacle_distance',
            px4_qos
        )

        self.get_logger().info(
            f'ObstacleAvoidanceCP ready: topic={self.DEPTH_TOPIC}, '
            f'HFOV={self.HFOV:.1f} deg, cover_bins={self.NUM_COVER}, '
            f'angle_offset={self.ANGLE_OFFSET_DEG:.1f} deg, '
            f'D_MIN={self.D_MIN} cm, D_MAX={self.D_MAX} cm'
        )

    def depth_cb(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}', throttle_duration_sec=1.0)
            return

        depth_mm = np.asarray(depth, dtype=np.float32)

        if depth_mm.ndim != 2:
            self.get_logger().warn(
                f'Unexpected depth image shape: {depth_mm.shape}',
                throttle_duration_sec=1.0
            )
            return

        h, w = depth_mm.shape
        if h < 2 or w < self.NUM_COVER:
            self.get_logger().warn(
                f'Depth image too small: {w}x{h}',
                throttle_duration_sec=1.0
            )
            return

        r_lo = max(0, min(h - 1, int(h * self.ROW_LO)))
        r_hi = max(r_lo + 1, min(h, int(h * self.ROW_HI)))
        roi = depth_mm[r_lo:r_hi, :]

        distances = [65535] * self.NUM_BINS  # unknown by default

        col_edges = np.linspace(0, w, self.NUM_COVER + 1, dtype=int)

        for i in range(self.NUM_COVER):
            c0 = int(col_edges[i])
            c1 = int(col_edges[i + 1])

            if c1 <= c0:
                continue

            col_slice = roi[:, c0:c1]
            total_pixels = col_slice.size
            if total_pixels == 0:
                continue

            # Robust validity filtering
            valid = col_slice[np.isfinite(col_slice)]
            valid = valid[(valid > self.D_MIN * 10.0) & (valid < self.D_MAX * 10.0)]

            valid_count = int(valid.size)
            valid_ratio = float(valid_count) / float(total_pixels)

            if valid_count < self.MIN_VALID_POINTS or valid_ratio < self.MIN_VALID_RATIO:
                continue

            d_cm = int(np.percentile(valid, self.PCNTL) / 10.0)
            d_cm = int(np.clip(d_cm, self.D_MIN, self.D_MAX))

            # Fill continuously at the start of the distances array.
            # Angle interpretation is handled by angle_offset + increment.
            distances[i] = d_cm

        obs = ObstacleDistance()
        obs.timestamp = self.get_clock().now().nanoseconds // 1000
        obs.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        obs.distances = distances
        obs.increment = float(self.BIN_DEG)
        obs.min_distance = int(self.D_MIN)
        obs.max_distance = int(self.D_MAX)
        obs.angle_offset = float(math.radians(self.ANGLE_OFFSET_DEG))
        obs.frame = ObstacleDistance.MAV_FRAME_BODY_FRD

        self.pub.publish(obs)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceCP()
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