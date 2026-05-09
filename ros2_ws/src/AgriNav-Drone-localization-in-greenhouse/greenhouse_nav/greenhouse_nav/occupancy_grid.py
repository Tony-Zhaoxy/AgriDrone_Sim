"""
occupancy_grid.py
=================
Converts RealSense depth images into a 2D local occupancy grid for
DWA-based obstacle avoidance.

This node is part of the flight safety pipeline:

  D455 depth image
      ↓
  project depth pixels into body/world frame
      ↓
  build /local_occupancy_grid
      ↓
  DWA planner uses the grid to avoid obstacles

Important:
  - This node requires a valid depth topic.
  - For real D455 flight, the expected depth topic is usually:
        /d455/d455/depth/image_rect_raw
  - It also requires PX4 local position:
        /fmu/out/vehicle_local_position
  - If no depth image is received, DWA will stay in WAIT_GRID and should not publish motion.

Topic map:
  SUB  /d455/d455/depth/image_rect_raw      sensor_msgs/Image
  SUB  /fmu/out/vehicle_local_position     px4_msgs/VehicleLocalPosition
  PUB  /local_occupancy_grid               nav_msgs/OccupancyGrid
"""

import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid as OccGrid
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy.ndimage import binary_dilation
from sensor_msgs.msg import Image


class OccupancyGridBuilder(Node):
    def __init__(self):
        super().__init__('occupancy_grid_builder')

        # Topics
        self.declare_parameter('depth_topic', '/d455/d455/depth/image_rect_raw')
        self.declare_parameter('position_topic', '/fmu/out/vehicle_local_position')

        # Grid parameters
        self.declare_parameter('grid_resolution_m', 0.10)
        self.declare_parameter('grid_size_m', 10.0)

        # IMPORTANT:
        # Smaller default inflation for corridor traversal.
        # At 0.10 m resolution, 0.20 m => 2 cells
        self.declare_parameter('inflation_radius_m', 0.20)

        # Depth range
        self.declare_parameter('min_depth_m', 0.20)
        self.declare_parameter('max_depth_m', 6.00)

        # Pole / thin obstacle help
        # Default OFF for greenhouse corridor traversal
        self.declare_parameter('use_min_depth_per_col', False)

        # If min-depth-per-column is enabled, only use a center image band
        # to reduce side-leaf overblocking
        self.declare_parameter('min_col_band_row_frac', 0.20)

        # Camera tilt angle, positive means pitched downward
        self.declare_parameter('camera_tilt_deg', 0.0)

        # Height band in BODY frame (z up)
        # Keep obstacles near drone height / in front, while rejecting floor/high clutter
        self.declare_parameter('height_low_m', -1.60)
        self.declare_parameter('height_high_m', 0.20)

        # Optional ROI on image rows
        self.declare_parameter('row_frac_lo', 0.10)
        self.declare_parameter('row_frac_hi', 0.90)

        # Depth camera intrinsics
        self.declare_parameter('fx', 432.5)
        self.declare_parameter('fy', 432.5)
        self.declare_parameter('cx', 319.5)
        self.declare_parameter('cy', 239.5)

        depth_topic = str(self.get_parameter('depth_topic').value)
        position_topic = str(self.get_parameter('position_topic').value)

        res = float(self.get_parameter('grid_resolution_m').value)
        size = float(self.get_parameter('grid_size_m').value)

        self.GRID_RES = res
        self.GRID_SIZE = size
        self.N = int(round(size / res))

        self.INFL_R = max(
            0,
            int(round(float(self.get_parameter('inflation_radius_m').value) / res))
        )
        self.D_MIN_M = float(self.get_parameter('min_depth_m').value)
        self.D_MAX_M = float(self.get_parameter('max_depth_m').value)
        self.USE_MIN = bool(self.get_parameter('use_min_depth_per_col').value)
        self.MIN_COL_BAND_FRAC = float(self.get_parameter('min_col_band_row_frac').value)

        self.TILT = math.radians(float(self.get_parameter('camera_tilt_deg').value))
        self.H_LOW = float(self.get_parameter('height_low_m').value)
        self.H_HIGH = float(self.get_parameter('height_high_m').value)

        self.ROW_LO = float(self.get_parameter('row_frac_lo').value)
        self.ROW_HI = float(self.get_parameter('row_frac_hi').value)

        self.FX = float(self.get_parameter('fx').value)
        self.FY = float(self.get_parameter('fy').value)
        self.CX = float(self.get_parameter('cx').value)
        self.CY = float(self.get_parameter('cy').value)

        self.bridge = CvBridge()
        self._disk = self._make_disk(self.INFL_R)

        # Vehicle state (world frame)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.yaw = 0.0
        self.have_pose = False
        self.drone_alt = 1.5  # positive up estimate, from NED z

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            Image,
            depth_topic,
            self.depth_cb,
            10
        )

        self.create_subscription(
            VehicleLocalPosition,
            position_topic,
            self.pos_cb,
            px4_qos
        )

        self.grid_pub = self.create_publisher(OccGrid, '/local_occupancy_grid', 10)

        self.get_logger().info(
            f'OccupancyGrid: topic={depth_topic}, {self.N}x{self.N} cells @ {res:.2f} m, '
            f'inflation={self.INFL_R} cells, min_col={self.USE_MIN}, '
            f'tilt={math.degrees(self.TILT):.1f} deg'
        )

    # ────────────────────────────────────────────────────────────── #
    # Pose callback
    # ────────────────────────────────────────────────────────────── #
    def pos_cb(self, msg: VehicleLocalPosition):
        self.pos_x = float(msg.x)
        self.pos_y = float(msg.y)
        self.pos_z = float(msg.z)
        self.yaw = float(msg.heading)
        self.drone_alt = -float(msg.z)  # NED z -> positive up
        self.have_pose = True

    # ────────────────────────────────────────────────────────────── #
    # Depth callback
    # ────────────────────────────────────────────────────────────── #
    def depth_cb(self, msg: Image):
        if not self.have_pose:
            self.get_logger().warn(
                'Waiting for vehicle pose before building occupancy grid...',
                throttle_duration_sec=2.0
            )
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}', throttle_duration_sec=1.0)
            return

        depth = np.asarray(depth)

        # Support both RealSense-like uint16(mm) and Gazebo float32(m)
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)

        # Clean invalid values
        depth_m[~np.isfinite(depth_m)] = 0.0
        depth_m[depth_m < self.D_MIN_M] = 0.0
        depth_m[depth_m > self.D_MAX_M] = 0.0

        grid, origin_x, origin_y = self._project_to_world_grid(depth_m)
        grid = self._inflate(grid)
        self._publish(grid, origin_x, origin_y)

    # ────────────────────────────────────────────────────────────── #
    # Core projection
    # ────────────────────────────────────────────────────────────── #
    def _project_to_world_grid(self, depth_m: np.ndarray):
        h, w = depth_m.shape
        grid = np.zeros((self.N, self.N), dtype=np.int8)

        # Local map centered around current vehicle position, but expressed in WORLD frame
        origin_x = self.pos_x - self.GRID_SIZE / 2.0
        origin_y = self.pos_y - self.GRID_SIZE / 2.0

        cos_t = math.cos(self.TILT)
        sin_t = math.sin(self.TILT)

        # Yaw rotation body -> world
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)

        row_lo = max(0, min(h - 1, int(h * self.ROW_LO)))
        row_hi = max(row_lo + 1, min(h, int(h * self.ROW_HI)))

        d_roi = depth_m[row_lo:row_hi, :]

        # Vectorized pixel coordinates
        u_arr = np.tile(np.arange(w), row_hi - row_lo)
        v_arr = np.repeat(np.arange(row_lo, row_hi), w)
        d_flat = d_roi.flatten()

        valid = (d_flat > self.D_MIN_M) & (d_flat < self.D_MAX_M)
        u_v = u_arr[valid]
        v_v = v_arr[valid]
        d_v = d_flat[valid]

        # Camera frame convention assumed:
        #   x_cam: right
        #   y_cam: down
        #   z_cam: forward
        x_cam = (u_v - self.CX) / self.FX * d_v
        y_cam = (v_v - self.CY) / self.FY * d_v
        z_cam = d_v

        # Rotate camera -> body using pitch tilt.
        # Body convention used here:
        #   x_body: forward
        #   y_body: left
        #   z_body: up
        x_body = cos_t * z_cam + sin_t * y_cam
        z_body = -sin_t * z_cam + cos_t * y_cam
        y_body = -x_cam

        # Height filter in body frame
        h_filter = (z_body > self.H_LOW) & (z_body < self.H_HIGH)
        x_f = x_body[h_filter]
        y_f = y_body[h_filter]

        # Optional per-column minimum depth:
        # Modified to be much less conservative than before.
        # Only use a center band in image rows and apply same height filtering.
        if self.USE_MIN:
            band_half = max(1, int(h * self.MIN_COL_BAND_FRAC / 2.0))
            band_center = int(self.CY)
            band_lo = max(0, band_center - band_half)
            band_hi = min(h, band_center + band_half + 1)

            d_band = depth_m[band_lo:band_hi, :]
            valid_band = (d_band > self.D_MIN_M) & (d_band < self.D_MAX_M)

            if np.any(valid_band):
                d_masked = np.where(valid_band, d_band, np.inf)
                col_mins = np.min(d_masked, axis=0)

                col_idx_arr = np.arange(w)
                finite_mask = np.isfinite(col_mins)
                col_d = col_mins[finite_mask]
                col_idx = col_idx_arr[finite_mask]

                # Use image-center row approximation
                v_center = np.full_like(col_idx, fill_value=self.CY, dtype=np.float32)

                x_cam_p = (col_idx - self.CX) / self.FX * col_d
                y_cam_p = (v_center - self.CY) / self.FY * col_d
                z_cam_p = col_d

                x_body_p = cos_t * z_cam_p + sin_t * y_cam_p
                z_body_p = -sin_t * z_cam_p + cos_t * y_cam_p
                y_body_p = -x_cam_p

                h_filter_p = (z_body_p > self.H_LOW) & (z_body_p < self.H_HIGH)
                x_body_p = x_body_p[h_filter_p]
                y_body_p = y_body_p[h_filter_p]

                if x_body_p.size > 0:
                    x_f = np.concatenate([x_f, x_body_p])
                    y_f = np.concatenate([y_f, y_body_p])

        if x_f.size == 0:
            return grid, origin_x, origin_y

        # Body -> world
        x_world = self.pos_x + cos_y * x_f - sin_y * y_f
        y_world = self.pos_y + sin_y * x_f + cos_y * y_f

        # World -> grid cell
        gx = ((x_world - origin_x) / self.GRID_RES).astype(int)
        gy = ((y_world - origin_y) / self.GRID_RES).astype(int)

        ok = (gx >= 0) & (gx < self.N) & (gy >= 0) & (gy < self.N)

        # OccupancyGrid is row-major: data[y * width + x]
        grid[gy[ok], gx[ok]] = 100

        return grid, origin_x, origin_y

    # ────────────────────────────────────────────────────────────── #
    # Inflation
    # ────────────────────────────────────────────────────────────── #
    def _inflate(self, grid: np.ndarray) -> np.ndarray:
        if self.INFL_R <= 0:
            return grid
        inflated = binary_dilation(grid == 100, structure=self._disk)
        result = np.zeros_like(grid)
        result[inflated] = 100
        return result

    @staticmethod
    def _make_disk(r: int) -> np.ndarray:
        if r <= 0:
            return np.ones((1, 1), dtype=bool)
        y, x = np.ogrid[-r:r + 1, -r:r + 1]
        return (x ** 2 + y ** 2 <= r ** 2)

    # ────────────────────────────────────────────────────────────── #
    # Publish
    # ────────────────────────────────────────────────────────────── #
    def _publish(self, grid: np.ndarray, origin_x: float, origin_y: float):
        msg = OccGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = float(self.GRID_RES)
        msg.info.width = int(self.N)
        msg.info.height = int(self.N)

        # Origin is the lower-left corner of the grid in WORLD frame
        msg.info.origin.position.x = float(origin_x)
        msg.info.origin.position.y = float(origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid.flatten().tolist()
        self.grid_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()