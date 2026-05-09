"""
marker_detector.py
==================
Detects ArUco / AprilTag markers from the D455 RGB camera and publishes
semantic mission events consumed by mission_executor and vio_bridge.

Marker ID convention (all configurable in mission.yaml):
  marker_id_init        (default  0) — VIO initialisation / mission-start gate
  marker_id_uturn_left  (default  1) — end-of-row: U-turn by rotating LEFT
  marker_id_uturn_right (default  2) — end-of-row: U-turn by rotating RIGHT
  marker_id_land        (default 10) — mission complete: stop and land

Both ArUco (e.g. DICT_4X4_50) and AprilTag (e.g. DICT_APRILTAG_36h11)
dictionaries are supported — select via the `aruco_dict` parameter.

Topic map:
  SUB  /d455/color/image_raw     (sensor_msgs/Image)
  PUB  /marker/event             (std_msgs/String)   — JSON event payload
  PUB  /marker/debug_image       (sensor_msgs/Image) — annotated frame (lazy)
"""

import json

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


# Half-size marker corners in marker-local frame (z=0 plane, metres)
# Order matches OpenCV's convention: TL, TR, BR, BL
_CORNER_TEMPLATE = np.array([
    [-0.5,  0.5, 0.0],
    [ 0.5,  0.5, 0.0],
    [ 0.5, -0.5, 0.0],
    [-0.5, -0.5, 0.0],
], dtype=np.float32)


class MarkerDetector(Node):
    """Detects ArUco/AprilTag markers and publishes semantic mission events."""

    def __init__(self):
        super().__init__('marker_detector')

        # ── Marker ID parameters ─────────────────────────────────────── #
        self.declare_parameter('marker_id_init',        0)
        self.declare_parameter('marker_id_uturn_left',  1)
        self.declare_parameter('marker_id_uturn_right', 2)
        self.declare_parameter('marker_id_land',       10)

        # ── Physical / detection parameters ──────────────────────────── #
        self.declare_parameter('marker_size_m', 0.15)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('detection_cooldown_sec', 2.0)
        self.declare_parameter('min_consecutive', 2)

        # ── Camera intrinsics (D455 colour camera defaults) ─────────── #
        self.declare_parameter('fx', 383.204)
        self.declare_parameter('fy', 383.204)
        self.declare_parameter('cx', 319.521)
        self.declare_parameter('cy', 237.854)

        self.ID_INIT = int(self.get_parameter('marker_id_init').value)
        self.ID_LEFT = int(self.get_parameter('marker_id_uturn_left').value)
        self.ID_RIGHT = int(self.get_parameter('marker_id_uturn_right').value)
        self.ID_LAND = int(self.get_parameter('marker_id_land').value)

        self.MARKER_SIZE = float(self.get_parameter('marker_size_m').value)
        self.COOLDOWN = float(self.get_parameter('detection_cooldown_sec').value)
        self.MIN_CONSEC = int(self.get_parameter('min_consecutive').value)

        fx = float(self.get_parameter('fx').value)
        fy = float(self.get_parameter('fy').value)
        cx = float(self.get_parameter('cx').value)
        cy = float(self.get_parameter('cy').value)

        self.camera_matrix = np.array([
            [fx,  0.0, cx],
            [0.0, fy,  cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float64)

        # Object-space corners scaled to the actual marker size
        self._obj_pts = (_CORNER_TEMPLATE * self.MARKER_SIZE).astype(np.float32)

        # ── ArUco / AprilTag detector ───────────────────────────────── #
        dict_name = str(self.get_parameter('aruco_dict').value)
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(
                getattr(cv2.aruco, dict_name)
            )
        except AttributeError:
            self.get_logger().error(
                f"Unknown aruco_dict '{dict_name}'. Falling back to DICT_4X4_50."
            )
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # Compatible with both old and new OpenCV ArUco APIs
        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_aruco_api = True
        else:
            self.detector = None
            self.use_new_aruco_api = False

        # ── ROS I/O ─────────────────────────────────────────────────── #
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self.create_subscription(
            Image,
            '/d455/color/image_raw',
            self.image_cb,
            img_qos,
        )

        self.event_pub = self.create_publisher(String, '/marker/event', 10)
        self.debug_pub = self.create_publisher(Image, '/marker/debug_image', 2)

        self.bridge = CvBridge()

        # ── State ───────────────────────────────────────────────────── #
        self._last_event_time = {}
        self._consec_count = {}

        self.get_logger().info(
            f'MarkerDetector ready | dict={dict_name} | size={self.MARKER_SIZE}m | '
            f'IDs: init={self.ID_INIT} left={self.ID_LEFT} '
            f'right={self.ID_RIGHT} land={self.ID_LAND}'
        )

    # ────────────────────────────────────────────────────────────────── #
    # Image callback
    # ────────────────────────────────────────────────────────────────── #
    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(
                f'cv_bridge conversion failed: {e}',
                throttle_duration_sec=2.0,
            )
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.use_new_aruco_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params
            )

        now = self.get_clock().now().nanoseconds * 1e-9

        # Collect which events were detected THIS frame
        detected_events = set()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            ids_flat = ids.flatten()

            for i, raw_id in enumerate(ids_flat):
                event = self._id_to_event(int(raw_id))
                if event is None:
                    continue

                # Per-event pose estimate (IPPE_SQUARE is optimal for square targets)
                ok, rvec, tvec = cv2.solvePnP(
                    self._obj_pts,
                    corners[i][0],
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                dist = float(np.linalg.norm(tvec)) if ok else float('nan')

                # Annotate frame with distance
                if ok:
                    cv2.drawFrameAxes(
                        frame,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        self.MARKER_SIZE * 0.5,
                    )
                    cx_px = int(corners[i][0][:, 0].mean())
                    cy_px = int(corners[i][0][:, 1].mean())
                    cv2.putText(
                        frame,
                        f'ID={raw_id} {event.replace("MARKER_", "")} {dist:.2f}m',
                        (cx_px - 60, cy_px - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )

                detected_events.add((event, int(raw_id), dist))

        # Update consecutive counters and publish events
        all_event_names = {e for e, _, _ in detected_events}

        # Decay counts for events not detected this frame
        for key in list(self._consec_count.keys()):
            if key not in all_event_names:
                self._consec_count[key] = 0

        for event, raw_id, dist in detected_events:
            self._consec_count[event] = self._consec_count.get(event, 0) + 1

            if self._consec_count[event] < self.MIN_CONSEC:
                continue

            last = self._last_event_time.get(event, 0.0)
            if (now - last) < self.COOLDOWN:
                continue

            self._last_event_time[event] = now
            self._publish_event(event, raw_id, dist)

        # Publish debug image only when someone is subscribed
        if self.debug_pub.get_subscription_count() > 0:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except Exception:
                pass

    # ────────────────────────────────────────────────────────────────── #
    # Helpers
    # ────────────────────────────────────────────────────────────────── #
    def _id_to_event(self, marker_id: int):
        if marker_id == self.ID_INIT:
            return 'MARKER_INIT'
        if marker_id == self.ID_LEFT:
            return 'MARKER_UTURN_LEFT'
        if marker_id == self.ID_RIGHT:
            return 'MARKER_UTURN_RIGHT'
        if marker_id == self.ID_LAND:
            return 'MARKER_LAND'
        return None

    def _publish_event(self, event: str, marker_id: int, dist: float):
        payload = json.dumps({
            'event': event,
            'marker_id': marker_id,
            'distance_m': round(dist, 3) if np.isfinite(dist) else None,
        })
        msg = String()
        msg.data = payload
        self.event_pub.publish(msg)
        self.get_logger().info(
            f'Marker event: {event}  id={marker_id}  dist={dist:.2f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()