#!/usr/bin/env python3

"""Standalone Vision1 object-detection preview window using current bartender topics."""

import os
import signal
import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_WEIGHTS = os.path.join(PROJECT_ROOT, "assets", "models", "cam_1.pt")
IMAGE_TOPICS = (
    "/camera/camera/color/image_raw",
    "/camera/camera_1/color/image_raw",
)
DEPTH_TOPICS = (
    "/camera/camera/aligned_depth_to_color/image_raw",
    "/camera/camera_1/aligned_depth_to_color/image_raw",
    "/camera/camera_1/depth/image_rect_raw",
)


class DrinkDetectionPreview(Node):
    def __init__(self):
        super().__init__("drink_detection_preview")
        self.depth_image = None
        self.color_image = None
        self.depth_scale = 0.001
        self.model = YOLO(DEFAULT_WEIGHTS)
        self._window_name = "Vision1 Standalone Preview"

        self._color_subs = [self.create_subscription(Image, topic, self.color_callback, 10) for topic in IMAGE_TOPICS]
        self._depth_subs = [self.create_subscription(Image, topic, self.depth_callback, 10) for topic in DEPTH_TOPICS]
        self.create_timer(0.25, self._show_waiting_frame)
        self.get_logger().info(f"preview images={list(IMAGE_TOPICS)} depths={list(DEPTH_TOPICS)}")

    def _show_waiting_frame(self):
        if self.color_image is not None:
            return
        canvas = np.zeros((520, 900, 3), dtype=np.uint8)
        cv2.putText(canvas, "Waiting for image...", (250, 260), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (220, 220, 220), 2, cv2.LINE_AA)
        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window_name, 900, 520)
        cv2.imshow(self._window_name, canvas)
        cv2.waitKey(1)

    def _decode_color(self, msg):
        h = int(msg.height)
        w = int(msg.width)
        step = int(msg.step)
        if h <= 0 or w <= 0:
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if str(msg.encoding).lower() not in ("rgb8", "bgr8"):
            return None
        need = w * 3
        if step < need or buf.size < h * step:
            return None
        arr = buf.reshape((h, step))[:, :need].reshape((h, w, 3))
        if str(msg.encoding).lower() == "rgb8":
            arr = arr[:, :, ::-1]
        return np.ascontiguousarray(arr)

    def _decode_depth(self, msg):
        h = int(msg.height)
        w = int(msg.width)
        step = int(msg.step)
        if h <= 0 or w <= 0:
            return None
        if str(msg.encoding).lower() not in ("16uc1", "mono16"):
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        need = w * 2
        if step < need or buf.size < h * step:
            return None
        arr = buf.reshape((h, step))[:, :need].reshape((h, w, 2))
        depth = arr[:, :, 0].astype(np.uint16) | (arr[:, :, 1].astype(np.uint16) << 8)
        return np.ascontiguousarray(depth)

    def color_callback(self, msg):
        frame = self._decode_color(msg)
        if frame is not None:
            self.color_image = frame

    def depth_callback(self, msg):
        depth = self._decode_depth(msg)
        if depth is None or self.color_image is None:
            return
        self.depth_image = depth

        depth_image = cv2.flip(self.depth_image.astype(np.float32) * self.depth_scale, -1)
        img_vis = cv2.flip(self.color_image.copy(), -1)
        results = self.model.predict(source=img_vis, conf=0.5, verbose=False)

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                try:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    confidence = float(box.conf[0].cpu().numpy())
                    class_id = int(box.cls[0].cpu().numpy())
                except Exception:
                    continue
                if confidence < 0.5:
                    continue
                roi = depth_image[y1:y2, x1:x2]
                valid = roi[np.isfinite(roi) & (roi > 0)]
                object_depth = float(np.median(valid)) if valid.size else float("nan")
                label = f"{self.model.names[int(class_id)]}/{object_depth:.2f}m"
                cv2.rectangle(img_vis, (x1, y1), (x2, y2), (252, 119, 30), 2)
                cv2.putText(
                    img_vis,
                    label,
                    (x1, max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (252, 119, 30),
                    2,
                    cv2.LINE_AA,
                )

        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window_name, 900, 520)
        cv2.imshow(self._window_name, img_vis)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DrinkDetectionPreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    raise SystemExit(main(sys.argv[1:]))
