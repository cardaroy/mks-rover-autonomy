#!/usr/bin/env python3
import json
import time
import os
from typing import List, Dict, Optional

import numpy as np
import cv2
from ultralytics import YOLO

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class CubeDetectorNode(Node):
    def __init__(self):
        super().__init__("cube_detector_node")

        # ===== ROS params =====
        DEFAULT_MODEL_PATH = os.environ.get("CUBE_MODEL_PATH", "")
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("conf_thres", 0.6)
        self.declare_parameter("depth_min_m", 0.15)
        self.declare_parameter("depth_max_m", 5.0)
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("publish_rate_hz", 15.0)  # reduce load

        # input topics (from realsense2_camera)
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

        # depth unit handling (RealSense ROS depth is usually in mm for 16UC1)
        self.declare_parameter("depth_unit", "mm")  # "mm" or "m"

        # ===== read params =====
        self.model_path = self.get_parameter("model_path").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.depth_min_m = float(self.get_parameter("depth_min_m").value)
        self.depth_max_m = float(self.get_parameter("depth_max_m").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.depth_unit = self.get_parameter("depth_unit").value  # "mm" or "m"

        # ===== guard =====
        if not self.model_path:
            raise RuntimeError(
                "Parameter 'model_path' is empty. Set it in launch or via CUBE_MODEL_PATH env var."
            )
        if not os.path.isfile(self.model_path):
            raise RuntimeError(f"Model not found: {self.model_path}")

        # ===== YOLO =====
        self.get_logger().info(f"Loading model: {self.model_path}")
        self.model = YOLO(self.model_path)

        # ===== ROS image bridge & buffers =====
        self.bridge = CvBridge()
        self._color_bgr: Optional[np.ndarray] = None
        self._depth_raw: Optional[np.ndarray] = None

        # intrinsics from CameraInfo
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None

        # ===== Subscribers =====
        self.color_sub = self.create_subscription(Image, self.color_topic, self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, 10)

        self.get_logger().info(
            f"Subscribed to:\n"
            f"  color: {self.color_topic}\n"
            f"  depth: {self.depth_topic}\n"
            f"  info : {self.camera_info_topic}\n"
            f"  depth_unit: {self.depth_unit}"
        )

        # ===== Publishers =====
        self.pub_pose_array = self.create_publisher(PoseArray, "/cube_poses", 10)
        self.pub_json = self.create_publisher(String, "/cube_detections_json", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/cube_markers", 10)

        # timer
        period = 1.0 / max(1e-3, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info("CubeDetectorNode started (ROS-topic mode).")

    def destroy_node(self):
        super().destroy_node()

    # --------- callbacks ---------
    def color_callback(self, msg: Image):
        # try bgr8 first; if fails, fallback to raw then convert
        try:
            self._color_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # if it's RGB, convert to BGR; otherwise keep
            if img.ndim == 3 and img.shape[2] == 3:
                self._color_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                self._color_bgr = img

    def depth_callback(self, msg: Image):
        # keep raw depth; convert to meters in tick()
        self._depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def info_callback(self, msg: CameraInfo):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    # --------- main loop ---------
    def tick(self):
        # need color + depth + intrinsics
        if self._color_bgr is None or self._depth_raw is None:
            return
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return

        color_bgr = self._color_bgr

        # depth to meters
        depth = self._depth_raw
        if depth is None:
            return

        depth_img = depth.astype(np.float32)
        if self.depth_unit.lower() == "mm":
            depth_img *= 0.001  # mm -> m
        # else assume already meters

        # 2) YOLO detect
        results = self.model(color_bgr, conf=self.conf_thres, verbose=False)
        r0 = results[0]

        detections: List[Dict] = []
        poses = PoseArray()
        poses.header.stamp = self.get_clock().now().to_msg()
        poses.header.frame_id = self.frame_id

        markers = MarkerArray()

        if r0.boxes is not None and len(r0.boxes) > 0:
            h, w = depth_img.shape[:2]

            for i, box in enumerate(r0.boxes):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int).tolist()
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                cls_name = self.model.names.get(cls_id, str(cls_id))

                # clamp bbox
                x1 = max(0, min(x1, w - 1))
                x2 = max(0, min(x2, w - 1))
                y1 = max(0, min(y1, h - 1))
                y2 = max(0, min(y2, h - 1))
                if x2 <= x1 or y2 <= y1:
                    continue

                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                # ROI median depth
                win = 9
                xs = max(0, u - win // 2)
                xe = min(w, u + win // 2 + 1)
                ys = max(0, v - win // 2)
                ye = min(h, v + win // 2 + 1)

                roi = depth_img[ys:ye, xs:xe]
                valid = roi[(roi > self.depth_min_m) & (roi < self.depth_max_m)]
                if valid.size == 0:
                    continue

                Z = float(np.median(valid))
                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy

                det = {
                    "class": cls_name,
                    "conf": round(conf, 4),
                    "bbox_xyxy": [int(x1), int(y1), int(x2), int(y2)],
                    "uv": [u, v],
                    "xyz_m": [round(X, 4), round(Y, 4), round(Z, 4)],
                }
                detections.append(det)

                # pose
                p = Pose()
                p.position.x = float(X)
                p.position.y = float(Y)
                p.position.z = float(Z)
                p.orientation.w = 1.0
                poses.poses.append(p)

                # marker: sphere
                m = Marker()
                m.header = poses.header
                m.ns = "cubes"
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose = p
                m.scale.x = 0.06
                m.scale.y = 0.06
                m.scale.z = 0.06
                m.color.a = 1.0
                markers.markers.append(m)

                # marker: text
                t = Marker()
                t.header = poses.header
                t.ns = "cube_text"
                t.id = 1000 + i
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose = p
                t.pose.position.z += 0.10
                t.scale.z = 0.08
                t.color.a = 1.0
                t.text = f"{cls_name} {conf:.2f}\nZ={Z:.2f}m"
                markers.markers.append(t)

        # publish
        self.pub_pose_array.publish(poses)

        msg = String()
        msg.data = json.dumps(
            {"stamp": time.time(), "frame_id": self.frame_id, "detections": detections},
            ensure_ascii=False,
        )
        self.pub_json.publish(msg)
        self.pub_markers.publish(markers)


def main():
    rclpy.init()
    node = CubeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()