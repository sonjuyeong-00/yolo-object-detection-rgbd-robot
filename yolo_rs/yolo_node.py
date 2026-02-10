#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header, Float32
from sensor_msgs_py import point_cloud2

from ultralytics import YOLO


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.bridge = CvBridge()

        # 학습된 모델 경로
        self.model = YOLO('/home/sson/ros2_ws/src/yolo_rs/dataset/runs/detect/train/weights/best.pt')

        # Topics
        self.color_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        self.info_topic  = '/camera/camera/color/camera_info'

        # Subscribers
        self.sub_color = self.create_subscription(Image, self.color_topic, self.image_callback, 10)
        self.sub_depth = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.sub_info  = self.create_subscription(CameraInfo, self.info_topic, self.info_callback, 10)

        # Publish: 모든 물체 3D 점들 (PointCloud2)
        self.pub_cloud = self.create_publisher(PointCloud2, '/snack_boxes/points', 10)

        # Publish: 선택된 물체의 3D point + gripper width
        self.pub_selected_point = self.create_publisher(PointStamped, '/snack_boxes/selected_point', 10)
        self.pub_selected_width = self.create_publisher(Float32, '/snack_boxes/selected_width', 10)

        # Tracker 설정
        self.tracker_cfg = "bytetrack.yaml"  # or "botsort.yaml"

        # 선택 상태
        self.selected_idx = None
        self.last_selected_time = None
        self.selection_timeout_sec = 5.0
        self.last_published_idx = None

        # UI용: 마지막 프레임 bbox 저장
        self.last_dets = []   # [(idx, x1, y1, x2, y2), ...]
        self.window_name = "YOLO"

        # OpenCV 창/마우스 콜백 등록 (imshow와 이름 동일해야 함)
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        # Cache
        self.depth_image = None
        self.depth_encoding = None  # '16UC1' or '32FC1'
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_frame_id = None

        self.get_logger().info(f'YOLO subscribed to {self.color_topic}')
        self.get_logger().info(f'Depth subscribed to {self.depth_topic}')
        self.get_logger().info(f'CameraInfo subscribed to {self.info_topic}')
        self.get_logger().info('Publishing all object points to /snack_boxes/points')
        self.get_logger().info('Publishing selected object to /snack_boxes/selected_point and /snack_boxes/selected_width')
        self.get_logger().info('Select by pressing number keys (0-9) OR mouse click in the OpenCV window. Press c to clear.')

        try:
            self.get_logger().info(f"Model names: {self.model.names}")
        except Exception:
            pass

    def set_selected_idx(self, idx: int):
        self.selected_idx = idx
        self.last_selected_time = self.get_clock().now()

    def clear_selection(self):
        self.selected_idx = None
        self.last_selected_time = None
        self.last_published_idx = None

    # CameraInfo 콜백
    def info_callback(self, msg: CameraInfo):
        k = msg.k
        self.fx = float(k[0])
        self.fy = float(k[4])
        self.cx = float(k[2])
        self.cy = float(k[5])
        self.camera_frame_id = msg.header.frame_id

    # Depth 콜백
    def depth_callback(self, msg: Image):
        self.depth_encoding = msg.encoding
        self.depth_image = self.bridge.imgmsg_to_cv2(msg)

    # bbox 내부 유효 depth median (m)
    def depth_in_bbox(self, x1: int, y1: int, x2: int, y2: int):
        if self.depth_image is None:
            return None

        h, w = self.depth_image.shape[:2]
        x1 = max(0, min(x1, w - 1))
        x2 = max(0, min(x2, w - 1))
        y1 = max(0, min(y1, h - 1))
        y2 = max(0, min(y2, h - 1))
        if x2 <= x1 or y2 <= y1:
            return None

        roi = self.depth_image[y1:y2, x1:x2].astype(np.float32)
        valid = roi[roi > 0]
        if valid.size < 50:
            return None

        z = float(np.median(valid))

        if self.depth_encoding == '16UC1':
            return z / 1000.0
        elif self.depth_encoding == '32FC1':
            return z
        else:
            return z / 1000.0

    # ✅ 마우스 클릭으로 bbox 선택
    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for idx, x1, y1, x2, y2 in self.last_dets:
                if x1 <= x <= x2 and y1 <= y <= y2:
                    self.set_selected_idx(idx)
                    self.get_logger().info(f"Selected by click: N={idx}")
                    break

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.depth_image is None or self.fx is None:
            cv2.putText(frame, "Waiting depth/camera_info...", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            # ✅ window_name 통일
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)
            return

        # Tracking
        results = self.model.track(
            source=frame,
            persist=True,
            tracker=self.tracker_cfg,
            conf=0.05,
            iou=0.45,
            verbose=False
        )[0]

        h, w = frame.shape[:2]
        img_area = w * h

        dets = []  # (u, v, x1,y1,x2,y2, conf, track_id)
        if results.boxes is not None:
            for b in results.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                conf = float(b.conf[0])

                track_id = None
                if getattr(b, "id", None) is not None and b.id is not None:
                    track_id = int(b.id[0])

                bw = max(1, x2 - x1)
                bh = max(1, y2 - y1)
                area = bw * bh

                if area > 0.35 * img_area:
                    continue
                if area < 0.0005 * img_area:
                    continue

                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)
                dets.append((u, v, x1, y1, x2, y2, conf, track_id))

        dets.sort(key=lambda t: (t[0], t[1]))  # 좌->우

        # ✅ 클릭 선택을 위해 bbox 목록 저장
        self.last_dets = []
        for idx, (_u, _v, x1, y1, x2, y2, _conf, _tid) in enumerate(dets):
            self.last_dets.append((idx, x1, y1, x2, y2))

        if self.selected_idx is not None and self.last_selected_time is not None:
            now = self.get_clock().now()
            elapsed_ns = (now - self.last_selected_time).nanoseconds
            if elapsed_ns >= int(self.selection_timeout_sec * 1e9):
                self.clear_selection()

        points_xyz = []
        selected_xyz = None
        selected_width_m = None

        for idx, (u, v, x1, y1, x2, y2, conf, track_id) in enumerate(dets):
            bw = max(1, x2 - x1)
            bh = max(1, y2 - y1)

            mx1 = int(x1 + 0.2 * bw)
            mx2 = int(x2 - 0.2 * bw)
            my1 = int(y1 + 0.2 * bh)
            my2 = int(y2 - 0.2 * bh)
            Z = self.depth_in_bbox(mx1, my1, mx2, my2)

            is_selected = (self.selected_idx == idx)
            color = (0, 255, 255) if is_selected else (0, 255, 0)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            if Z is None:
                label = f'N:{idx} ID:{track_id} {conf:.2f} Z:None' if track_id is not None else f'N:{idx} {conf:.2f} Z:None'
                cv2.putText(frame, label, (x1, max(15, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                continue

            X = (u - self.cx) * Z / self.fx
            Y = (v - self.cy) * Z / self.fy
            points_xyz.append((float(X), float(Y), float(Z)))

            width_m = float(bw * Z / self.fx)

            cv2.circle(frame, (u, v), 4, color, -1)
            label = (f'N:{idx} ID:{track_id} {conf:.2f} Z:{Z:.2f}m W:{width_m*1000:.0f}mm'
                     if track_id is not None else
                     f'N:{idx} {conf:.2f} Z:{Z:.2f}m W:{width_m*1000:.0f}mm')
            cv2.putText(frame, label, (x1, max(15, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            if is_selected:
                selected_xyz = (float(X), float(Y), float(Z))
                selected_width_m = width_m

        # 모든 점 publish
        if points_xyz:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.camera_frame_id if self.camera_frame_id else 'camera_color_optical_frame'
            cloud = point_cloud2.create_cloud_xyz32(header, points_xyz)
            self.pub_cloud.publish(cloud)

        # 선택된 물체 publish (선택 변화 시 1회만)
        if selected_xyz is not None and self.selected_idx != self.last_published_idx:
            X, Y, Z = selected_xyz
            ps = PointStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = self.camera_frame_id if self.camera_frame_id else 'camera_color_optical_frame'
            ps.point.x = X
            ps.point.y = Y
            ps.point.z = Z
            self.pub_selected_point.publish(ps)

            if selected_width_m is not None:
                self.pub_selected_width.publish(Float32(data=float(selected_width_m)))

            self.last_published_idx = self.selected_idx

        info1 = "Click bbox or press 0-9 to select N:index | Press c to clear"
        info2 = f"Selected: {self.selected_idx}" if self.selected_idx is not None else "Selected: None"
        cv2.putText(frame, info1, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, info2, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow(self.window_name, frame)

        key = cv2.waitKey(1) & 0xFF
        if ord('0') <= key <= ord('9'):
            self.set_selected_idx(int(chr(key)))
        elif key == ord('c'):
            self.clear_selection()


def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
