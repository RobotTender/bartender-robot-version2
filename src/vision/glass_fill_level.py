#!/usr/bin/env python3

"""Vision2 glass fill level metadata publisher for bartender-robot."""

import os
import sys

import cv2
import numpy as np
from ultralytics import YOLO


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from vision_meta_common import BaseVisionMetaProcess, build_parser, model_path, run_process, simplify_contour


DEFAULT_IMAGE_TOPIC = os.environ.get("VISION2_IMAGE_TOPIC", "/camera2/camera/color/image_raw")
DEFAULT_DEPTH_TOPIC = os.environ.get("VISION2_DEPTH_TOPIC", "/camera2/camera/aligned_depth_to_color/image_raw")
DEFAULT_OUTPUT_TOPIC = os.environ.get("VISION_VOLUME_META_TOPIC_2", "/vision2/volume/meta")
DEFAULT_WEIGHTS = model_path("cam_2.pt")


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


class GlassFillLevelProcess(BaseVisionMetaProcess):
    def __init__(self, args):
        self.mode = "volume"
        self.weights_path = os.path.abspath(str(args.weights or "").strip() or DEFAULT_WEIGHTS)
        _enable_legacy_model_aliases()
        self.model = YOLO(self.weights_path)
        self.conf = float(args.conf) if float(args.conf) > 0.0 else 0.25
        self.height_ema_alpha = 0.2
        self.height_px_ema = None
        self.fixed_bottle_bottom_y = None
        self.bottle_class_name = "bottle"
        self.known_heights_px = np.array([0, 71, 105, 140, 180, 206, 253, 290, 338, 400, 450], dtype=np.float32)
        self.known_volumes_ml = np.array([0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500], dtype=np.float32)
        super().__init__(args, f"vision{int(args.panel)}_volume_meta")
        self.get_logger().info(f"mode={self.mode} weights={self.weights_path} conf={self.conf:.2f}")

    def _height_px_to_volume_ml(self, height_px: float):
        return float(np.interp(height_px, self.known_heights_px, self.known_volumes_ml))

    def _apply_height_ema(self, value: float):
        if value is None:
            return self.height_px_ema
        if self.height_px_ema is None:
            self.height_px_ema = float(value)
        else:
            self.height_px_ema = (
                self.height_ema_alpha * float(value)
                + (1.0 - self.height_ema_alpha) * float(self.height_px_ema)
            )
        return float(self.height_px_ema)

    def build_payload(self, frame):
        src_h, src_w = frame.shape[:2]
        frame_infer = self._rotate_image(frame)
        depth_saved = self.latest_depth
        depth_infer = self._rotate_image(depth_saved) if depth_saved is not None else None
        results = self.model.predict(
            source=frame_infer,
            conf=self.conf,
            iou=0.5,
            retina_masks=True,
            verbose=False,
        )
        frame_h, frame_w = frame_infer.shape[:2]
        detections = []
        bottle = None
        liquid = None
        bottle_mask_current = None
        liquid_mask_current = None
        bottle_bbox_current = None
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
            for i, box in enumerate(boxes):
                try:
                    confidence = float(box.conf[0].cpu().numpy())
                    class_id = int(box.cls[0].cpu().numpy())
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                except Exception:
                    continue
                mi = mdata[i]
                if mi.shape != (frame_h, frame_w):
                    mi = cv2.resize(mi, (frame_w, frame_h), interpolation=cv2.INTER_NEAREST)
                mask_bin = (mi > 0.5).astype(np.uint8)
                mask_area = int(np.count_nonzero(mask_bin))
                class_name = str(self.model.names.get(class_id, class_id))
                contour = simplify_contour(mask_bin)
                depth_m = self._mask_depth_m_from_array(depth_infer, mask_bin)
                bbox_src = self._map_bbox_rotated_to_source((x1, y1, x2, y2), src_w, src_h)
                contour_src = self._map_contour_rotated_to_source(contour, src_w, src_h)
                det = {
                    "class_id": class_id,
                    "class_name": class_name,
                    "confidence": confidence,
                    "bbox_xyxy": [int(v) for v in bbox_src],
                    "center_uv": [
                        int(round((float(bbox_src[0]) + float(bbox_src[2])) / 2.0)),
                        int(round((float(bbox_src[1]) + float(bbox_src[3])) / 2.0)),
                    ],
                    "mask_area": mask_area,
                    "depth_m": float(depth_m) if depth_m is not None else None,
                    "contour_uv": contour_src,
                }
                detections.append(det)
                if class_name == self.bottle_class_name:
                    if bottle is None or int(mask_area) > int(bottle.get("mask_area", 0)):
                        bottle = det
                        bottle_mask_current = mask_bin.copy()
                        bottle_bbox_current = (x1, y1, x2, y2)
                else:
                    if liquid is None or int(mask_area) > int(liquid.get("mask_area", 0)):
                        liquid = det
                        liquid_mask_current = mask_bin.copy()
                        liquid_bbox_current = (x1, y1, x2, y2)

        volume_ml = None
        waterline_y = None
        height_px = None
        height_px_ema = None
        bottom_y = self.fixed_bottle_bottom_y

        if bottle_mask_current is not None and liquid_mask_current is not None and self.fixed_bottle_bottom_y is None:
            ys_b = np.where(bottle_mask_current > 0)[0]
            if len(ys_b) > 0:
                self.fixed_bottle_bottom_y = int(np.max(ys_b))
                bottom_y = self.fixed_bottle_bottom_y

        if bottle_mask_current is not None and liquid_mask_current is not None and self.fixed_bottle_bottom_y is not None:
            ys_l = np.where(liquid_mask_current > 0)[0]
            if len(ys_l) > 0:
                waterline_y = int(np.min(ys_l))
                height_px = int(self.fixed_bottle_bottom_y - waterline_y)
                if height_px >= 0:
                    height_px_ema = self._apply_height_ema(height_px)
                    volume_ml = self._height_px_to_volume_ml(height_px_ema)
                bottom_y = self.fixed_bottle_bottom_y

        if liquid is not None:
            liquid = dict(liquid)
            if waterline_y is not None and liquid_bbox_current is not None:
                lx1, _ly1, lx2, _ly2 = [int(v) for v in liquid_bbox_current]
                p1 = self._map_rotated_to_source_coords(lx1, int(waterline_y), src_w, src_h)
                p2 = self._map_rotated_to_source_coords(lx2, int(waterline_y), src_w, src_h)
                liquid["waterline_segment_uv"] = [[int(p1[0]), int(p1[1])], [int(p2[0]), int(p2[1])]]
                liquid["waterline_y"] = int(round((p1[1] + p2[1]) / 2.0))
            else:
                liquid["waterline_segment_uv"] = None
                liquid["waterline_y"] = None
            liquid["bottom_y"] = int(bottom_y) if bottom_y is not None else None
            liquid["height_px"] = int(height_px) if height_px is not None else None
            liquid["height_px_ema"] = float(height_px_ema) if height_px_ema is not None else None
            liquid["volume_ml"] = float(volume_ml) if volume_ml is not None else None
        if bottle is not None:
            bottle = dict(bottle)
            bottle["bottom_y"] = int(bottom_y) if bottom_y is not None else None

        detections.sort(key=lambda item: float(item.get("confidence", 0.0)), reverse=True)
        return {
            "panel": self.panel,
            "mode": self.mode,
            "detected": bool(detections),
            "volume_ready": volume_ml is not None,
            "frame_size": [int(src_w), int(src_h)],
            "model_path": self.weights_path,
            "detections": detections,
            "bottle": bottle,
            "liquid": liquid,
        }


def parse_args(argv=None):
    parser = build_parser(
        description="Vision2 glass fill level metadata publisher",
        default_panel=2,
        default_image_topic=DEFAULT_IMAGE_TOPIC,
        default_depth_topic=DEFAULT_DEPTH_TOPIC,
        default_output_meta_topic=DEFAULT_OUTPUT_TOPIC,
    )
    return parser.parse_args(argv)


def main(args=None):
    opts = parse_args(args)
    return run_process(GlassFillLevelProcess, opts, args=args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
