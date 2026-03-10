#!/usr/bin/env python3

"""Vision1 drink object detection metadata publisher for bartender-robot."""

import os
import sys

from ultralytics import YOLO


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from vision_meta_common import BaseVisionMetaProcess, build_parser, model_path, run_process


DEFAULT_IMAGE_TOPIC = os.environ.get("VISION1_IMAGE_TOPIC", "/camera/camera/color/image_raw")
DEFAULT_DEPTH_TOPIC = os.environ.get("VISION1_DEPTH_TOPIC", "/camera/camera/aligned_depth_to_color/image_raw")
DEFAULT_OUTPUT_TOPIC = os.environ.get("VISION_OBJECT_META_TOPIC_1", "/vision1/object/meta")
DEFAULT_WEIGHTS = model_path("cam_1.pt")


class DrinkDetectionProcess(BaseVisionMetaProcess):
    def __init__(self, args):
        self.mode = "object"
        self.weights_path = os.path.abspath(str(args.weights or "").strip() or DEFAULT_WEIGHTS)
        self.model = YOLO(self.weights_path)
        self.conf = float(args.conf) if float(args.conf) > 0.0 else 0.5
        super().__init__(args, f"vision{int(args.panel)}_object_meta")
        self.get_logger().info(f"mode={self.mode} weights={self.weights_path} conf={self.conf:.2f}")

    def build_payload(self, frame):
        src_h, src_w = frame.shape[:2]
        frame_infer = self._rotate_image(frame)
        depth_rot = self._rotate_image(self.latest_depth) if self.latest_depth is not None else None
        results = self.model.predict(source=frame_infer, conf=self.conf, verbose=False)
        detections = []
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
                bbox_src = self._map_bbox_rotated_to_source((x1, y1, x2, y2), src_w, src_h)
                depth_m = self._depth_median_m_from_array(depth_rot, x1, y1, x2, y2)
                detections.append(
                    {
                        "class_id": class_id,
                        "class_name": str(self.model.names.get(class_id, class_id)),
                        "confidence": confidence,
                        "bbox_xyxy": [int(v) for v in bbox_src],
                        "center_uv": [
                            int(round((float(bbox_src[0]) + float(bbox_src[2])) / 2.0)),
                            int(round((float(bbox_src[1]) + float(bbox_src[3])) / 2.0)),
                        ],
                        "depth_m": float(depth_m) if depth_m is not None else None,
                    }
                )
        detections.sort(key=lambda item: float(item.get("confidence", 0.0)), reverse=True)
        return {
            "panel": self.panel,
            "mode": self.mode,
            "detected": bool(detections),
            "frame_size": [int(src_w), int(src_h)],
            "model_path": self.weights_path,
            "detections": detections,
        }


def parse_args(argv=None):
    parser = build_parser(
        description="Vision1 drink object detection metadata publisher",
        default_panel=1,
        default_image_topic=DEFAULT_IMAGE_TOPIC,
        default_depth_topic=DEFAULT_DEPTH_TOPIC,
        default_output_meta_topic=DEFAULT_OUTPUT_TOPIC,
    )
    return parser.parse_args(argv)


def main(args=None):
    opts = parse_args(args)
    return run_process(DrinkDetectionProcess, opts, args=args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
