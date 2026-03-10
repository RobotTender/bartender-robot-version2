#!/usr/bin/env python3

"""Standalone Vision2 fill-level preview window using current bartender topics."""

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
DEFAULT_WEIGHTS = os.path.join(PROJECT_ROOT, "assets", "models", "cam_2.pt")
IMAGE_TOPICS = (
    "/camera2/camera/color/image_raw",
    "/camera/camera_2/color/image_raw",
)
DEPTH_TOPICS = (
    "/camera2/camera/aligned_depth_to_color/image_raw",
    "/camera/camera_2/aligned_depth_to_color/image_raw",
    "/camera/camera_2/depth/image_rect_raw",
)
CLASS_COLORS = {
    0: (0, 0, 255),
    1: (255, 0, 0),
    2: (0, 255, 0),
}


def _enable_legacy_model_aliases():
    # Older segmentation checkpoints may serialize custom class names.
    # Map them to current ultralytics module symbols for compatibility.
    try:
        import ultralytics.nn.modules.head as _head
        if (not hasattr(_head, "Segment26")) and hasattr(_head, "Segment"):
            _head.Segment26 = _head.Segment
    except Exception:
        pass
    try:
        import ultralytics.nn.modules.block as _block
        if (not hasattr(_block, "Proto26")) and hasattr(_block, "Proto"):
            _block.Proto26 = _block.Proto
    except Exception:
        pass


class GlassFillLevelPreview(Node):
    def __init__(self):
        super().__init__("glass_fill_level_preview")
        self.depth_image = None
        self.color_image = None
        self.depth_scale = 0.001
        _enable_legacy_model_aliases()
        self.model = YOLO(DEFAULT_WEIGHTS)
        self.bottle_class_name = "bottle"
        self.known_heights_px = np.array([0, 71, 105, 140, 180, 206, 253, 290, 338, 400, 450], dtype=np.float32)
        self.known_volumes_ml = np.array([0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500], dtype=np.float32)
        self.height_ema_alpha = 0.2
        self.height_px_ema = None
        self.fixed_bottle_bottom_y = None
        self._window_name = "Vision2 Standalone Preview"

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

    def height_px_to_volume_ml(self, height_px: float):
        return float(np.interp(height_px, self.known_heights_px, self.known_volumes_ml))

    def apply_height_ema(self, value: float):
        if value is None:
            return self.height_px_ema
        if self.height_px_ema is None:
            self.height_px_ema = float(value)
        else:
            self.height_px_ema = self.height_ema_alpha * float(value) + (1.0 - self.height_ema_alpha) * float(self.height_px_ema)
        return float(self.height_px_ema)

    def depth_callback(self, msg):
        depth = self._decode_depth(msg)
        if depth is None or self.color_image is None:
            return
        self.depth_image = depth

        depth_m = cv2.flip(self.depth_image.astype(np.float32) * self.depth_scale, -1)
        img_vis = cv2.flip(self.color_image.copy(), -1)
        results = self.model.predict(source=img_vis, conf=0.25, iou=0.5, retina_masks=True, verbose=False)
        overlay = img_vis.copy()

        bottle_mask_current = None
        liquid_mask_current = None
        liquid_bbox_current = None

        for result in results:
            boxes = result.boxes
            masks = result.masks
            if boxes is None or masks is None or len(boxes) == 0:
                continue
            try:
                mdata = masks.data.detach().cpu().numpy()
            except Exception:
                mdata = np.asarray(masks.data)
            h, w = img_vis.shape[:2]
            for i, box in enumerate(boxes):
                try:
                    conf = float(box.conf[0].cpu().numpy())
                    cls_id = int(box.cls[0].cpu().numpy())
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                except Exception:
                    continue
                mi = mdata[i]
                if mi.shape != (h, w):
                    mi = cv2.resize(mi, (w, h), interpolation=cv2.INTER_NEAREST)
                mask_bin = (mi > 0.5).astype(np.uint8)
                mask_area = int(np.count_nonzero(mask_bin))
                class_name = self.model.names.get(cls_id, cls_id)
                color = CLASS_COLORS.get(cls_id, (255, 255, 255))

                if class_name == self.bottle_class_name:
                    if bottle_mask_current is None or mask_area > int(np.count_nonzero(bottle_mask_current)):
                        bottle_mask_current = mask_bin.copy()
                else:
                    if liquid_mask_current is None or mask_area > int(np.count_nonzero(liquid_mask_current)):
                        liquid_mask_current = mask_bin.copy()
                        liquid_bbox_current = (x1, y1, x2, y2)

                mask_alpha = 0.2
                colored = np.zeros_like(overlay, dtype=np.uint8)
                colored[:, :] = color
                overlay = np.where(
                    mask_bin[:, :, None].astype(bool),
                    ((1 - mask_alpha) * overlay + mask_alpha * colored).astype(np.uint8),
                    overlay,
                )

                label = f"{class_name} {conf:.2f}"
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    overlay,
                    label,
                    (x1, max(18, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    color,
                    2,
                    cv2.LINE_AA,
                )

        if bottle_mask_current is not None and liquid_mask_current is not None and self.fixed_bottle_bottom_y is None:
            ys_b = np.where(bottle_mask_current > 0)[0]
            if len(ys_b) > 0:
                self.fixed_bottle_bottom_y = int(np.max(ys_b))

        if bottle_mask_current is not None and liquid_mask_current is not None and self.fixed_bottle_bottom_y is not None:
            ys_l = np.where(liquid_mask_current > 0)[0]
            if len(ys_l) > 0:
                waterline_y = int(np.min(ys_l))
                height_px = int(self.fixed_bottle_bottom_y - waterline_y)
                if height_px >= 0 and liquid_bbox_current is not None:
                    volume_ml = self.height_px_to_volume_ml(self.apply_height_ema(height_px))
                    _lx1, ly1, lx2, _ly2 = liquid_bbox_current
                    cv2.putText(
                        overlay,
                        f"volume={volume_ml:.1f}ml",
                        (lx2 - 140, max(18, ly1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )

        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window_name, 900, 520)
        cv2.imshow(self._window_name, overlay)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = GlassFillLevelPreview()
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
