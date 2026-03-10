#!/usr/bin/env python3

import argparse
import json
import os
import signal
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
MODEL_DIR = os.path.join(PROJECT_ROOT, "assets", "models")


def model_path(filename: str) -> str:
    return os.path.join(MODEL_DIR, str(filename))


def build_parser(
    description,
    default_panel=1,
    default_image_topic="",
    default_depth_topic="",
    default_output_meta_topic="",
):
    parser = argparse.ArgumentParser(description=str(description))
    parser.add_argument("--panel", type=int, default=int(default_panel))
    parser.add_argument("--image-topic", default=str(default_image_topic), required=not bool(default_image_topic))
    parser.add_argument("--depth-topic", default=str(default_depth_topic), required=not bool(default_depth_topic))
    parser.add_argument(
        "--output-meta-topic",
        default=str(default_output_meta_topic),
        required=not bool(default_output_meta_topic),
    )
    parser.add_argument("--weights", default="")
    parser.add_argument("--process-hz", type=float, default=8.0)
    parser.add_argument("--conf", type=float, default=-1.0)
    parser.add_argument("--rotation-deg", type=int, default=0)
    return parser


def simplify_contour(mask_bin: np.ndarray):
    try:
        contours, _ = cv2.findContours(mask_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    except Exception:
        return None
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if contour is None or len(contour) < 3:
        return None
    peri = float(cv2.arcLength(contour, True))
    epsilon = max(1.0, peri * 0.0025)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    pts = approx.reshape(-1, 2) if approx is not None else contour.reshape(-1, 2)
    if pts.shape[0] > 240:
        step = int(np.ceil(float(pts.shape[0]) / 240.0))
        pts = pts[::step]
    out = []
    for x, y in pts:
        out.append([int(x), int(y)])
    return out if len(out) >= 3 else None


class BaseVisionMetaProcess(Node):
    def __init__(self, args, node_name: str):
        super().__init__(str(node_name))
        self.args = args
        self.panel = 2 if int(args.panel) == 2 else 1
        self.rotation_deg = self._normalize_rotation_deg(int(args.rotation_deg))
        self.depth_encoding = ""
        self.latest_color = None
        self.latest_depth = None
        self.latest_seq = 0
        self.last_processed_seq = 0
        self.last_input_at = 0.0
        self.last_process_ms = None
        self._shutting_down = False

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.meta_pub = self.create_publisher(String, args.output_meta_topic, 10)
        self.create_subscription(Image, args.image_topic, self._on_image, qos)
        self.create_subscription(Image, args.depth_topic, self._on_depth, qos)
        self._tick_timer = self.create_timer(1.0 / max(1.0, float(args.process_hz)), self._tick)
        self.get_logger().info(
            f"panel={self.panel} image={args.image_topic} depth={args.depth_topic} "
            f"meta={args.output_meta_topic} rot={self.rotation_deg}"
        )

    def _normalize_rotation_deg(self, value):
        try:
            v = int(value)
        except Exception:
            v = 0
        v = v % 360
        candidates = (0, 90, 180, 270)
        return min(candidates, key=lambda x: abs(x - v))

    def _rotate_image(self, image):
        rot = self.rotation_deg
        if image is None or rot == 0:
            return image
        if rot == 90:
            return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        if rot == 180:
            return cv2.rotate(image, cv2.ROTATE_180)
        if rot == 270:
            return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return image

    def _map_rotated_to_source_coords(self, x_rot: int, y_rot: int, src_w: int, src_h: int):
        rot = self.rotation_deg
        x_r = int(max(0, min(max(0, (src_h - 1) if rot in (90, 270) else (src_w - 1)), x_rot)))
        y_r = int(max(0, min(max(0, (src_w - 1) if rot in (90, 270) else (src_h - 1)), y_rot)))
        if rot == 90:
            return int(max(0, min(src_w - 1, y_r))), int(max(0, min(src_h - 1, (src_h - 1) - x_r)))
        if rot == 180:
            return int(max(0, min(src_w - 1, (src_w - 1) - x_r))), int(max(0, min(src_h - 1, (src_h - 1) - y_r)))
        if rot == 270:
            return int(max(0, min(src_w - 1, (src_w - 1) - y_r))), int(max(0, min(src_h - 1, x_r)))
        return int(max(0, min(src_w - 1, x_r))), int(max(0, min(src_h - 1, y_r)))

    def _map_bbox_rotated_to_source(self, bbox_rot, src_w, src_h):
        x1, y1, x2, y2 = [int(v) for v in bbox_rot]
        corners = [
            self._map_rotated_to_source_coords(x1, y1, src_w, src_h),
            self._map_rotated_to_source_coords(x2, y1, src_w, src_h),
            self._map_rotated_to_source_coords(x2, y2, src_w, src_h),
            self._map_rotated_to_source_coords(x1, y2, src_w, src_h),
        ]
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        return [int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys))]

    def _map_contour_rotated_to_source(self, contour_uv, src_w, src_h):
        if not contour_uv:
            return None
        out = []
        for pt in contour_uv:
            if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                continue
            try:
                mapped = self._map_rotated_to_source_coords(int(round(float(pt[0]))), int(round(float(pt[1]))), src_w, src_h)
            except Exception:
                continue
            out.append([int(mapped[0]), int(mapped[1])])
        return out if len(out) >= 3 else None

    def request_shutdown(self):
        if self._shutting_down:
            return
        self._shutting_down = True
        try:
            self._tick_timer.cancel()
        except Exception:
            pass

    def _safe_publish(self, payload):
        if self._shutting_down:
            return False
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        try:
            self.meta_pub.publish(msg)
            return True
        except Exception:
            return False

    def _decode_color(self, msg):
        h = int(msg.height)
        w = int(msg.width)
        step = int(msg.step)
        if h <= 0 or w <= 0:
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        enc = str(msg.encoding).lower()
        if enc in ("rgb8", "bgr8"):
            need = w * 3
            if step < need or buf.size < h * step:
                return None
            arr = buf.reshape((h, step))[:, :need].reshape((h, w, 3))
            bgr = arr[:, :, ::-1] if enc == "rgb8" else arr
            return np.ascontiguousarray(bgr)
        if enc in ("rgba8", "bgra8"):
            need = w * 4
            if step < need or buf.size < h * step:
                return None
            arr = buf.reshape((h, step))[:, :need].reshape((h, w, 4))
            if enc == "bgra8":
                arr = arr[:, :, [2, 1, 0, 3]]
            return np.ascontiguousarray(arr[:, :, :3][:, :, ::-1])
        return None

    def _decode_depth(self, msg):
        h = int(msg.height)
        w = int(msg.width)
        step = int(msg.step)
        if h <= 0 or w <= 0:
            return None, ""
        enc = str(msg.encoding).lower()
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in ("16uc1", "mono16"):
            need = w * 2
            if step < need or buf.size < h * step:
                return None, ""
            arr = buf.reshape((h, step))[:, :need].reshape((h, w, 2))
            depth = arr[:, :, 0].astype(np.uint16) | (arr[:, :, 1].astype(np.uint16) << 8)
            return np.ascontiguousarray(depth), "16UC1"
        if enc == "32fc1":
            need = w * 4
            if step < need or buf.size < h * step:
                return None, ""
            depth = np.frombuffer(msg.data, dtype=np.float32, count=h * w).reshape((h, w))
            return np.ascontiguousarray(depth), "32FC1"
        return None, ""

    def _on_image(self, msg):
        frame = self._decode_color(msg)
        if frame is None:
            return
        self.latest_color = frame
        self.latest_seq += 1
        self.last_input_at = time.monotonic()

    def _on_depth(self, msg):
        depth, encoding = self._decode_depth(msg)
        if depth is None:
            return
        self.latest_depth = depth
        self.depth_encoding = encoding

    def _depth_median_m_from_array(self, depth_array, x1, y1, x2, y2):
        if depth_array is None:
            return None
        h, w = depth_array.shape[:2]
        x1 = max(0, min(w - 1, int(x1)))
        x2 = max(x1 + 1, min(w, int(x2)))
        y1 = max(0, min(h - 1, int(y1)))
        y2 = max(y1 + 1, min(h, int(y2)))
        roi = depth_array[y1:y2, x1:x2]
        if roi.size == 0:
            return None
        valid = roi[np.isfinite(roi) & (roi > 0)]
        if valid.size < 3:
            return None
        z = float(np.median(valid))
        if self.depth_encoding == "16UC1":
            z /= 1000.0
        if z < 0.05 or z > 5.0:
            return None
        return z

    def _mask_depth_m_from_array(self, depth_array, mask_bin):
        if depth_array is None or mask_bin is None:
            return None
        roi = depth_array[mask_bin.astype(bool)]
        valid = roi[np.isfinite(roi) & (roi > 0)]
        if valid.size < 3:
            return None
        z = float(np.median(valid))
        if self.depth_encoding == "16UC1":
            z /= 1000.0
        if z < 0.05 or z > 5.0:
            return None
        return z

    def build_payload(self, frame):
        raise NotImplementedError

    def _tick(self):
        if self._shutting_down:
            return
        # Some deployments may temporarily miss depth topics; run inference with color-only
        # and keep depth-derived fields as None instead of halting the whole metadata stream.
        if self.latest_color is None:
            return
        if self.latest_seq == self.last_processed_seq:
            return
        self.last_processed_seq = int(self.latest_seq)
        started = time.monotonic()
        frame = self.latest_color.copy()
        try:
            payload = self.build_payload(frame)
        except Exception as exc:
            payload = {
                "panel": self.panel,
                "detected": False,
                "error": str(exc),
                "frame_size": [int(frame.shape[1]), int(frame.shape[0])],
                "detections": [],
            }
        self.last_process_ms = (time.monotonic() - started) * 1000.0
        payload["processing_ms"] = float(self.last_process_ms)
        payload["source_age_ms"] = max(0.0, (time.monotonic() - float(self.last_input_at or time.monotonic())) * 1000.0)
        self._safe_publish(payload)


def run_process(node_factory, opts, args=None):
    rclpy.init(args=args)
    node = node_factory(opts)

    def _handle_signal(signum, _frame):
        _ = signum
        node.request_shutdown()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _handle_signal)
        except Exception:
            pass

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.request_shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0
