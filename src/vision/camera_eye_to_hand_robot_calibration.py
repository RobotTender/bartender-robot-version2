#!/usr/bin/env python3
import argparse
import json
import signal
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def parse_args():
    parser = argparse.ArgumentParser(description="External camera-eye-to-hand calibration processor")
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--depth-topic", required=True)
    parser.add_argument("--camera-info-topic", required=True)
    parser.add_argument("--output-image-topic", default="")
    parser.add_argument("--output-meta-topic", required=True)
    parser.add_argument("--cols", type=int, default=7)
    parser.add_argument("--rows", type=int, default=9)
    parser.add_argument("--panel", type=int, default=1)
    parser.add_argument("--hold-sec", type=float, default=1.2)
    parser.add_argument("--detect-interval-sec", type=float, default=0.18)
    parser.add_argument("--process-hz", type=float, default=10.0)
    parser.add_argument("--max-side", type=int, default=960)
    return parser.parse_args()


def detect_checkerboard(gray, cols, rows):
    candidate_sizes = []
    for c, r in (
        (cols, rows),
        (max(3, cols - 1), max(3, rows - 1)),
        (rows, cols),
        (max(3, rows - 1), max(3, cols - 1)),
    ):
        if (c, r) not in candidate_sizes:
            candidate_sizes.append((c, r))
    variants = [gray, cv2.equalizeHist(gray)]
    try:
        variants.append(cv2.GaussianBlur(gray, (5, 5), 0))
    except Exception:
        pass
    variants.append(255 - gray)
    for g in variants:
        for cand_cols, cand_rows in candidate_sizes:
            pattern_size = (cand_cols, cand_rows)
            if hasattr(cv2, "findChessboardCornersSB"):
                flags = cv2.CALIB_CB_NORMALIZE_IMAGE
                if hasattr(cv2, "CALIB_CB_EXHAUSTIVE"):
                    flags |= cv2.CALIB_CB_EXHAUSTIVE
                if hasattr(cv2, "CALIB_CB_ACCURACY"):
                    flags |= cv2.CALIB_CB_ACCURACY
                found, corners = cv2.findChessboardCornersSB(g, pattern_size, flags=flags)
                if found and corners is not None:
                    return corners.reshape(cand_rows, cand_cols, 2).astype(np.float32)
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            found, corners = cv2.findChessboardCorners(g, pattern_size, flags=flags)
            if found and corners is not None:
                term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
                corners = cv2.cornerSubPix(g, corners, (11, 11), (-1, -1), term)
                return corners.reshape(cand_rows, cand_cols, 2).astype(np.float32)
    return None


def board_outer_from_inner(pts_grid):
    rows, cols = pts_grid.shape[:2]
    p_tl = pts_grid[0, 0]
    p_tr = pts_grid[0, cols - 1]
    p_bl = pts_grid[rows - 1, 0]
    p_br = pts_grid[rows - 1, cols - 1]
    ux_top = (p_tr - p_tl) / max(1, cols - 1)
    ux_bot = (p_br - p_bl) / max(1, cols - 1)
    uy_left = (p_bl - p_tl) / max(1, rows - 1)
    uy_right = (p_br - p_tr) / max(1, rows - 1)
    ux = 0.5 * (ux_top + ux_bot)
    uy = 0.5 * (uy_left + uy_right)
    return p_tl - ux - uy, p_tr + ux - uy, p_bl - ux + uy, p_br + ux + uy


def order_outer_screen(points4):
    pts = [np.array(p, dtype=np.float32) for p in points4]
    pts_sorted = sorted(pts, key=lambda p: (float(p[1]), float(p[0])))
    top2 = sorted(pts_sorted[:2], key=lambda p: float(p[0]))
    bot2 = sorted(pts_sorted[2:], key=lambda p: float(p[0]))
    return top2[0], top2[1], bot2[0], bot2[1]


def line_intersection(a1, a2, b1, b2):
    x1, y1 = float(a1[0]), float(a1[1])
    x2, y2 = float(a2[0]), float(a2[1])
    x3, y3 = float(b1[0]), float(b1[1])
    x4, y4 = float(b2[0]), float(b2[1])
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-9:
        return None
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
    return np.array([px, py], dtype=np.float32)


def pick_p5_upper_right(pts_grid, center):
    pts = pts_grid.reshape(-1, 2)
    cx, cy = float(center[0]), float(center[1])
    candidates = [p for p in pts if float(p[0]) >= cx and float(p[1]) <= cy]
    if not candidates:
        candidates = [p for p in pts if float(p[0]) >= cx] or [p for p in pts if float(p[1]) <= cy] or list(pts)
    best = min(candidates, key=lambda p: (float(p[0] - cx) ** 2 + float(p[1] - cy) ** 2))
    return np.array(best, dtype=np.float32)


def draw_checkerboard_overlay(frame, points_dict, grid_points_uv, grid_status, full_ready, reason):
    annotated = np.ascontiguousarray(frame.copy())
    try:
        grid = np.asarray(grid_points_uv, dtype=np.float32)
    except Exception:
        grid = None
    if grid is not None and grid.ndim == 3 and grid.shape[0] >= 2 and grid.shape[1] >= 2:
        try:
            rows, cols = grid.shape[:2]
            expanded = np.zeros((rows + 2, cols + 2, 2), dtype=np.float32)
            expanded[1 : rows + 1, 1 : cols + 1] = grid
            expanded[0, 1 : cols + 1] = 1.5 * grid[0, :] - 0.5 * grid[1, :]
            expanded[rows + 1, 1 : cols + 1] = 1.5 * grid[rows - 1, :] - 0.5 * grid[rows - 2, :]
            expanded[:, 0] = 1.5 * expanded[:, 1] - 0.5 * expanded[:, 2]
            expanded[:, cols + 1] = 1.5 * expanded[:, cols] - 0.5 * expanded[:, cols - 1]
            for rr in range(expanded.shape[0]):
                for cc in range(expanded.shape[1] - 1):
                    p1 = expanded[rr, cc]
                    p2 = expanded[rr, cc + 1]
                    cv2.line(
                        annotated,
                        (int(round(float(p1[0]))), int(round(float(p1[1])))),
                        (int(round(float(p2[0]))), int(round(float(p2[1])))),
                        (0, 220, 0),
                        1,
                        cv2.LINE_AA,
                    )
            for cc in range(expanded.shape[1]):
                for rr in range(expanded.shape[0] - 1):
                    p1 = expanded[rr, cc]
                    p2 = expanded[rr + 1, cc]
                    cv2.line(
                        annotated,
                        (int(round(float(p1[0]))), int(round(float(p1[1])))),
                        (int(round(float(p2[0]))), int(round(float(p2[1])))),
                        (0, 220, 0),
                        1,
                        cv2.LINE_AA,
                    )
        except Exception:
            pass
    if isinstance(grid_status, list):
        for row in grid_status:
            if not isinstance(row, (list, tuple)) or len(row) < 3:
                continue
            try:
                u_f = float(row[0])
                v_f = float(row[1])
                ok_pt = bool(row[2])
            except Exception:
                continue
            color = (0, 220, 0) if ok_pt else (0, 0, 255)
            cv2.circle(annotated, (int(round(u_f)), int(round(v_f))), 2, color, -1, cv2.LINE_AA)
    center = None
    if isinstance(points_dict, dict):
        center = points_dict.get("center")
    if isinstance(center, (list, tuple)) and len(center) >= 2:
        cv2.circle(
            annotated,
            (int(round(float(center[0]))), int(round(float(center[1])))),
            4,
            (0, 0, 255),
            -1,
            cv2.LINE_AA,
        )
    color = (0, 200, 0) if bool(full_ready) else (0, 180, 255)
    cv2.putText(annotated, str(reason or "대기"), (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    return annotated


class CalibrationProcess(Node):
    def __init__(self, args):
        super().__init__(f"vision{int(args.panel)}_calibration")
        self.args = args
        self.cols = max(2, int(args.cols))
        self.rows = max(2, int(args.rows))
        self.hold_sec = max(0.1, float(args.hold_sec))
        self.detect_interval_sec = max(0.05, float(args.detect_interval_sec))
        self.process_hz = max(1.0, float(args.process_hz))
        self.max_side = max(320, int(args.max_side))

        self.last_depth = None
        self.depth_encoding = ""
        self.depth_shape = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.latest_frame = None
        self.latest_stamp = None
        self.latest_frame_seq = 0
        self.latest_frame_received_at = 0.0
        self.input_prev_at = None
        self.input_interval_ms = None
        self.last_processed_seq = 0
        self.last_result = None
        self.last_result_at = 0.0
        self.last_detect_run_at = 0.0
        self.last_process_ms = None
        self.publish_prev_at = None
        self.publish_interval_ms = None
        self._shutting_down = False

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.image_pub = self.create_publisher(Image, args.output_image_topic, qos) if str(args.output_image_topic).strip() else None
        self.meta_pub = self.create_publisher(String, args.output_meta_topic, 10)
        self.create_subscription(Image, args.image_topic, self._on_image, qos)
        self.create_subscription(Image, args.depth_topic, self._on_depth, qos)
        self.create_subscription(CameraInfo, args.camera_info_topic, self._on_camera_info, qos)
        self._tick_timer = self.create_timer(1.0 / self.process_hz, self._tick)
        self.get_logger().info(
            f"panel={args.panel} image={args.image_topic} depth={args.depth_topic} info={args.camera_info_topic}"
        )

    def request_shutdown(self):
        if self._shutting_down:
            return
        self._shutting_down = True
        try:
            if self._tick_timer is not None:
                self._tick_timer.cancel()
        except Exception:
            pass

    def _publisher_alive(self, publisher):
        if self._shutting_down or publisher is None:
            return False
        try:
            return bool(rclpy.ok())
        except Exception:
            return False

    def _safe_publish(self, publisher, msg):
        if not self._publisher_alive(publisher):
            return False
        try:
            publisher.publish(msg)
            return True
        except Exception:
            if self._shutting_down:
                return False
            try:
                if not rclpy.ok():
                    return False
            except Exception:
                return False
            raise

    def _on_camera_info(self, msg):
        try:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])
        except Exception:
            self.fx = self.fy = self.cx = self.cy = None

    def _on_depth(self, msg):
        try:
            h = int(msg.height)
            w = int(msg.width)
            step = int(msg.step)
            if h <= 0 or w <= 0:
                return
            enc = str(msg.encoding).upper()
            if enc == "16UC1":
                arr = np.frombuffer(msg.data, dtype=np.uint16)
                need = w * 2
                if step < need or arr.size < (h * step // 2):
                    return
                self.last_depth = arr.reshape((h, step // 2))[:, :w].copy()
            elif enc == "32FC1":
                arr = np.frombuffer(msg.data, dtype=np.float32)
                need = w * 4
                if step < need or arr.size < (h * step // 4):
                    return
                self.last_depth = arr.reshape((h, step // 4))[:, :w].copy()
            else:
                return
            self.depth_encoding = enc
            self.depth_shape = (h, w)
        except Exception:
            return

    def _on_image(self, msg):
        frame = self._decode_bgr(msg)
        if frame is None:
            return
        now = time.monotonic()
        if self.input_prev_at is not None:
            dt = now - self.input_prev_at
            if dt > 0.0:
                self.input_interval_ms = dt * 1000.0
        self.input_prev_at = now
        self.latest_frame = frame
        self.latest_stamp = msg.header.stamp
        self.latest_frame_seq += 1
        self.latest_frame_received_at = now

    def _depth_m(self, u, v):
        if self.last_depth is None or self.depth_shape is None:
            return None
        try:
            x = int(round(float(u)))
            y = int(round(float(v)))
            h, w = self.depth_shape
            if x < 0 or y < 0 or x >= w or y >= h:
                return None
            roi = self.last_depth[max(0, y - 2) : min(h, y + 3), max(0, x - 2) : min(w, x + 3)]
            valid = roi[np.isfinite(roi) & (roi > 0)]
            if valid.size < 3:
                return None
            z = float(np.median(valid))
            if self.depth_encoding == "16UC1":
                z /= 1000.0
            if z < 0.05 or z > 5.0:
                return None
            return z
        except Exception:
            return None

    def _uvz_to_xyz_mm(self, u, v, z_mm):
        if None in (self.fx, self.fy, self.cx, self.cy):
            return None
        z = float(z_mm)
        x = ((float(u) - self.cx) / self.fx) * z
        y = ((float(v) - self.cy) / self.fy) * z
        return [float(x), float(y), float(z)]

    def _decode_bgr(self, msg):
        h = int(msg.height)
        w = int(msg.width)
        step = int(msg.step)
        if h <= 0 or w <= 0:
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ("rgb8", "bgr8"):
            need = w * 3
            if step < need or buf.size < h * step:
                return None
            arr = buf.reshape((h, step))[:, :need].reshape((h, w, 3))
            return np.ascontiguousarray(arr[:, :, ::-1] if msg.encoding == "rgb8" else arr)
        if msg.encoding in ("rgba8", "bgra8"):
            need = w * 4
            if step < need or buf.size < h * step:
                return None
            arr = buf.reshape((h, step))[:, :need].reshape((h, w, 4))
            if msg.encoding == "bgra8":
                arr = arr[:, :, [2, 1, 0, 3]]
            return np.ascontiguousarray(arr[:, :, :3][:, :, ::-1])
        return None

    def _image_msg_from_bgr(self, frame, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "calibration"
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(frame.shape[1] * 3)
        msg.data = np.ascontiguousarray(frame).tobytes()
        return msg

    def _detect_result(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        scale = 1.0
        if max(h, w) > self.max_side:
            scale = float(self.max_side) / float(max(h, w))
            gray_small = cv2.resize(gray, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        else:
            gray_small = gray
        pts_grid = detect_checkerboard(gray_small, self.cols, self.rows)
        if pts_grid is None:
            return None
        if scale != 1.0:
            pts_grid = pts_grid / float(scale)

        o_tl, o_tr, o_bl, o_br = order_outer_screen(board_outer_from_inner(pts_grid))
        center = line_intersection(o_tl, o_br, o_tr, o_bl)
        if center is None:
            center = 0.25 * (o_tl + o_tr + o_bl + o_br)
        p5 = pick_p5_upper_right(pts_grid, center)
        names = [("p1", o_br), ("p2", o_bl), ("p3", o_tr), ("p4", o_tl), ("p5", p5)]
        points = {}
        invalid_depth_count = 0
        for name, pt in names:
            u = float(pt[0])
            v = float(pt[1])
            z_m = self._depth_m(u, v)
            z_mm = float("nan") if z_m is None else float(z_m * 1000.0)
            if z_m is None:
                invalid_depth_count += 1
            points[name] = [u, v, z_mm]
        center_z = self._depth_m(center[0], center[1])
        center_z_mm = float("nan") if center_z is None else float(center_z * 1000.0)
        points["center"] = [float(center[0]), float(center[1]), center_z_mm]

        grid_uv = np.asarray(pts_grid, dtype=np.float64)
        grid_status = []
        grid_xyz_rows = []
        bad_grid = 0
        for row in grid_uv:
            xyz_row = []
            for pt in row:
                u = float(pt[0])
                v = float(pt[1])
                z_m = self._depth_m(u, v)
                ok = False
                xyz = None
                if z_m is not None:
                    xyz = self._uvz_to_xyz_mm(u, v, float(z_m * 1000.0))
                    ok = xyz is not None and np.isfinite(np.asarray(xyz, dtype=np.float64)).all()
                if not ok:
                    bad_grid += 1
                grid_status.append([u, v, bool(ok)])
                xyz_row.append(xyz if ok else None)
            grid_xyz_rows.append(xyz_row)
        full_ready = int(grid_uv.shape[0] * grid_uv.shape[1]) > 0 and bad_grid == 0
        reason = "전체 보드 데이터 준비 완료" if full_ready else f"전체 코너 데이터 미수집({bad_grid}/{int(grid_uv.shape[0] * grid_uv.shape[1])})"
        full_reason = reason
        result = {
            "detected": True,
            "reason": reason,
            "full_ready": bool(full_ready),
            "full_reason": full_reason,
            "center": list(points["center"]),
            "points": points,
            "grid_points_uv": grid_uv.tolist(),
            "grid_status": grid_status,
            "grid_camera_xyz_mm": grid_xyz_rows if full_ready else None,
            "invalid_depth_count": int(invalid_depth_count),
        }
        return result

    def _build_result(self, frame):
        now = time.monotonic()
        detect_due = (self.last_result is None) or ((now - self.last_detect_run_at) >= self.detect_interval_sec)
        if detect_due:
            self.last_detect_run_at = now
            detected = self._detect_result(frame)
            if detected is not None:
                self.last_result = detected
                self.last_result_at = now
                return detected
        if self.last_result is not None and (now - self.last_result_at) <= self.hold_sec:
            held = json.loads(json.dumps(self.last_result))
            held["reason"] = "체커보드 추적중"
            return held
        return {
            "detected": False,
            "reason": "체커보드 미검출",
            "full_ready": False,
            "full_reason": "체커보드 미검출",
            "center": None,
            "points": None,
            "grid_points_uv": None,
            "grid_status": None,
            "grid_camera_xyz_mm": None,
            "invalid_depth_count": 0,
        }

    def _tick(self):
        if self._shutting_down:
            return
        if self.latest_frame is None or self.latest_stamp is None:
            return
        if self.latest_frame_seq == self.last_processed_seq:
            return
        tick_started_at = time.monotonic()
        frame = np.ascontiguousarray(self.latest_frame.copy())
        stamp = self.latest_stamp
        seq = int(self.latest_frame_seq)
        result = self._build_result(frame)
        process_done_at = time.monotonic()
        self.last_process_ms = (process_done_at - tick_started_at) * 1000.0
        if self.publish_prev_at is not None:
            publish_dt = process_done_at - self.publish_prev_at
            if publish_dt > 0.0:
                self.publish_interval_ms = publish_dt * 1000.0
        self.publish_prev_at = process_done_at
        payload = dict(result)
        payload["panel"] = int(self.args.panel)
        payload["stamp_sec"] = float(time.time())
        payload["raw_input_interval_ms"] = None if self.input_interval_ms is None else float(self.input_interval_ms)
        payload["processing_ms"] = None if self.last_process_ms is None else float(self.last_process_ms)
        payload["publish_interval_ms"] = None if self.publish_interval_ms is None else float(self.publish_interval_ms)
        payload["frame_wait_ms"] = None if self.latest_frame_received_at <= 0.0 else float((tick_started_at - self.latest_frame_received_at) * 1000.0)
        if self.image_pub is not None:
            annotated = draw_checkerboard_overlay(
                frame,
                result.get("points"),
                result.get("grid_points_uv"),
                result.get("grid_status"),
                result.get("full_ready"),
                result.get("reason"),
            )
            if not self._safe_publish(self.image_pub, self._image_msg_from_bgr(annotated, stamp)):
                return
        meta = String()
        meta.data = json.dumps(payload, ensure_ascii=False)
        if not self._safe_publish(self.meta_pub, meta):
            return
        self.last_processed_seq = seq


def main():
    args = parse_args()
    rclpy.init()
    node = CalibrationProcess(args)

    def _request_shutdown(*_args):
        node.request_shutdown()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    try:
        signal.signal(signal.SIGTERM, _request_shutdown)
        signal.signal(signal.SIGINT, _request_shutdown)
    except Exception:
        pass
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.request_shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
