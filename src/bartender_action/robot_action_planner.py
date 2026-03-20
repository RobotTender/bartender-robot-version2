"""Bartender robot action planner.

목적:
- 주문 결과(레시피) + vision1 객체 메타를 바탕으로 제조 시퀀스를 구성한다.
- 시퀀스는 robot_action_planner 한 곳에서만 정의한다.
- 네이티브 실행(api=NativeRobotActionApi)과 step 기반 실행(api=None)을 동시에 지원한다.

제조 개념:
1) 주문/레시피 해석
2) 비전1으로 병 타겟 계산
3) 병 파지 -> 따르기 -> 비전2 용량 피드백 -> 병 원위치
4) 다음 재료 반복
5) 완성컵 파지 -> 전달 -> 홈 복귀
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Any


# ---------------------------------------------------------------------------
# 1) 메뉴/레시피 규칙
# ---------------------------------------------------------------------------

MENU_ORDER = {
    "soju": ["soju"],
    "beer": ["beer"],
    "somaek": ["soju", "beer"],
}

MENU_DEFAULT_RECIPE = {
    "soju": {"soju": 50.0},
    "beer": {"beer": 200.0},
    "somaek": {"soju": 60.0, "beer": 140.0},
}

INGREDIENT_CLASS_ALIASES = {
    "soju": ("soju", "소주"),
    "beer": ("beer", "맥주"),
    "glass": ("glass", "cup", "잔", "컵"),
}

CUP_CLASS_ALIASES = INGREDIENT_CLASS_ALIASES["glass"]


# ---------------------------------------------------------------------------
# 2) 시퀀스 포즈/파라미터
# ---------------------------------------------------------------------------

# 중요:
# 아래 포즈/오프셋 상수는 "기본(fallback) 값"이다.
# 실제 실행값 우선순위는
# 1) context["robot_action_pose_config"] 2) config/robot_action_pose_config.json 3) 아래 상수
# 이다. 즉 저장/전달된 설정이 있으면 아래 값은 자동으로 대체된다.
SERVICE_READY_POSJ = [28.0, -35.0, 100.0, 77.0, 63.0, -154.0]
POUR_START_CHEERS_POSJ = [45.0, 0.0, 135.0, 90.0, -90.0, -135.0]
POUR_CONTACT_POSJ = [45.00, 43.58, 134.19, 90.01, -90.00, -62.23]
POUR_HORIZONTAL_POSJ = [42.43, 21.08, 129.85, 87.75, -88.75, -29.06]
POUR_DIAGONAL_POSJ = [41.83, -5.00, 134.35, 87.99, -87.55, -0.61]
POUR_VERTICAL_POSJ = [38.76, -35.80, 146.74, 87.76, -84.18, 22.06]
CUP_PICK_READY_POSJ = list(SERVICE_READY_POSJ)
CUP_DELIVERY_READY_POSJ = list(SERVICE_READY_POSJ)

# 완성컵 집기/전달(고정 절대좌표, mm/deg)
CUP_PICK_APPROACH_POSX = [430.0, -110.0, 300.0, 180.0, 0.0, 180.0]
CUP_PICK_POSE_POSX = [430.0, -110.0, 225.0, 180.0, 0.0, 180.0]
CUP_PICK_LIFT_POSX = [430.0, -110.0, 330.0, 180.0, 0.0, 180.0]
PICK_LIFT_OUT_POSX = [99.555, 533.322, 701.878, 88.252, 89.415, -91.978]

CUP_DELIVERY_APPROACH_POSX = [520.0, -20.0, 320.0, 180.0, 0.0, 180.0]
CUP_DELIVERY_POSE_POSX = [520.0, -20.0, 235.0, 180.0, 0.0, 180.0]

GRIPPER_OPEN_MM_DEFAULT = 109.0
GRIPPER_CLOSE_MM_DEFAULT = 41.0

# 실시간 따르기 제어 파라미터
LIVE_POUR_VOLUME_POLL_SEC = 0.01
LIVE_POUR_FINAL_WAIT_POLL_SEC = 0.01

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
ROBOT_ACTION_POSE_CONFIG_PATH = os.path.abspath(
    str(
        os.environ.get(
            "ROBOT_ACTION_POSE_CONFIG_PATH",
            os.path.join(PROJECT_ROOT, "config", "robot_action_pose_config.json"),
        )
        or os.path.join(PROJECT_ROOT, "config", "robot_action_pose_config.json")
    )
)

RUNTIME_POSE_DEFAULTS = {
    "service_ready_posj": list(SERVICE_READY_POSJ),
    "pour_start_cheers_posj": list(POUR_START_CHEERS_POSJ),
    "pour_contact_posj": list(POUR_CONTACT_POSJ),
    "pour_horizontal_posj": list(POUR_HORIZONTAL_POSJ),
    "pour_diagonal_posj": list(POUR_DIAGONAL_POSJ),
    "pour_vertical_posj": list(POUR_VERTICAL_POSJ),
    "cup_pick_ready_posj": list(CUP_PICK_READY_POSJ),
    "cup_pick_approach_posx": list(CUP_PICK_APPROACH_POSX),
    "cup_pick_pose_posx": list(CUP_PICK_POSE_POSX),
    "cup_pick_lift_posx": list(CUP_PICK_LIFT_POSX),
    "pick_lift_out_posx": list(PICK_LIFT_OUT_POSX),
    "cup_delivery_ready_posj": list(CUP_DELIVERY_READY_POSJ),
    "cup_delivery_approach_posx": list(CUP_DELIVERY_APPROACH_POSX),
    "cup_delivery_pose_posx": list(CUP_DELIVERY_POSE_POSX),
}

# runtime_cfg가 비어있거나 파일 로드 실패 시 사용하는 기본 오프셋
RUNTIME_OFFSET_DEFAULTS_XYZ_MM = {
    "pick_approach_offset": [0.0, -50.0, 0.0],
    "pick_grasp_offset": [0.0, 0.0, 0.0],
    "pick_lift_offset": [0.0, 0.0, 100.0],
    "place_offset": [0.0, 0.0, 0.0],
    "retreat_offset": [-20.0, -50.0, 0.0],
}


class PlannerSequenceApi:
    """플래너 호출형 API -> backend step 포맷 변환기.

    모션 메서드는 원본 DSR API 의미를 이름에 반영한다.
    - movel_posx(...): movel(posx(...), vel=..., acc=..., radius=..., ra=...)
    - movej_posj(...): movej(posj(...), v=..., a=...)
    - amovel_posx(...): amovel(posx(...), vel=..., acc=..., radius=..., ra=...)
    - amovej_posj(...): amovej(posj(...), v=..., a=..., radius=..., ra=...)
    """

    def __init__(self):
        self._steps = []

    @property
    def sequence_steps(self):
        return list(self._steps)

    def _append(self, step: dict):
        self._steps.append(dict(step))

    @staticmethod
    def _is_resolved_target_pose(pose):
        return isinstance(pose, dict) and ("__resolved_target_key__" in pose)

    def set_robot_mode(self, mode: int, label: str = "로봇 오토모드 전환", enabled: bool = True):
        self._append(
            {
                "op": "set_robot_mode",
                "label": str(label),
                "mode": int(mode),
                "enabled": bool(enabled),
            }
        )

    def move_home(self, label: str = "홈 이동", enabled: bool = True):
        self._append(
            {
                "op": "backend_call",
                "label": str(label),
                "method": "send_move_home",
                "args": [],
                "kwargs": {},
                "enabled": bool(enabled),
            }
        )

    def movel_posx(
        self,
        pose6,
        label: str = "카테시안 이동",
        enabled: bool = True,
        timeout_sec: float | None = None,
        vel: float | None = None,
        acc: float | None = None,
    ):
        if self._is_resolved_target_pose(pose6):
            payload = dict(pose6)
            step = {
                "op": "movel_resolved_target",
                "label": str(label),
                "target_key": str(payload.get("__resolved_target_key__", "")),
                "approach_up_mm": float(payload.get("approach_up_mm", 0.0)),
                "enabled": bool(enabled),
            }
            if payload.get("xyzabc") is not None:
                step["xyzabc"] = list(payload.get("xyzabc"))
            if payload.get("offset_xyz_mm") is not None:
                step["offset_xyz_mm"] = list(payload.get("offset_xyz_mm"))
            if payload.get("abc") is not None:
                step["abc"] = list(payload.get("abc"))
            if timeout_sec is not None:
                step["timeout_sec"] = float(timeout_sec)
            if vel is not None:
                step["vel"] = float(vel)
            if acc is not None:
                step["acc"] = float(acc)
            self._append(step)
            return

        vals = [float(v) for v in list(pose6)[:6]]
        kwargs = {}
        if vel is not None:
            kwargs["vel"] = float(vel)
        if acc is not None:
            kwargs["acc"] = float(acc)
        step = {
            "op": "backend_call",
            "label": str(label),
            "method": "send_move_cartesian",
            "args": vals,
            "kwargs": kwargs,
            "enabled": bool(enabled),
        }
        if timeout_sec is not None:
            step["timeout_sec"] = float(timeout_sec)
        self._append(step)

    def movej_posj(
        self,
        joints6,
        label: str = "조인트 이동",
        enabled: bool = True,
        timeout_sec: float | None = None,
        vel: float | None = None,
        acc: float | None = None,
    ):
        vals = [float(v) for v in list(joints6)[:6]]
        if len(vals) != 6:
            raise ValueError("movej_posj는 6개 조인트 값이 필요합니다.")
        kwargs = {}
        if vel is not None:
            kwargs["vel"] = float(vel)
        if acc is not None:
            kwargs["acc"] = float(acc)
        step = {
            "op": "backend_call",
            "label": str(label),
            "method": "send_move_joint",
            "args": vals,
            "kwargs": kwargs,
            "enabled": bool(enabled),
        }
        if timeout_sec is not None:
            step["timeout_sec"] = float(timeout_sec)
        self._append(step)

    def amovel_posx(
        self,
        pose6,
        label: str = "카테시안 비동기 이동",
        enabled: bool = True,
        timeout_sec: float | None = None,
        vel: float | None = None,
        acc: float | None = None,
    ):
        vals = [float(v) for v in list(pose6)[:6]]
        if len(vals) != 6:
            raise ValueError("amovel_posx는 6개 XYZABC 값이 필요합니다.")
        kwargs = {}
        if vel is not None:
            kwargs["vel"] = float(vel)
        if acc is not None:
            kwargs["acc"] = float(acc)
        step = {
            "op": "backend_call",
            "label": str(label),
            "method": "send_move_cartesian_async",
            "args": vals,
            "kwargs": kwargs,
            "enabled": bool(enabled),
        }
        if timeout_sec is not None:
            step["timeout_sec"] = float(timeout_sec)
        self._append(step)

    def amovej_posj(
        self,
        joints6,
        label: str = "조인트 비동기 이동",
        enabled: bool = True,
        timeout_sec: float | None = None,
        vel: float | None = None,
        acc: float | None = None,
    ):
        vals = [float(v) for v in list(joints6)[:6]]
        if len(vals) != 6:
            raise ValueError("amovej_posj는 6개 조인트 값이 필요합니다.")
        kwargs = {}
        if vel is not None:
            kwargs["vel"] = float(vel)
        if acc is not None:
            kwargs["acc"] = float(acc)
        step = {
            "op": "backend_call",
            "label": str(label),
            "method": "send_move_joint_async",
            "args": vals,
            "kwargs": kwargs,
            "enabled": bool(enabled),
        }
        if timeout_sec is not None:
            step["timeout_sec"] = float(timeout_sec)
        self._append(step)

    # backward compatibility aliases
    def movel(
        self,
        pose6,
        label: str = "카테시안 이동",
        enabled: bool = True,
        timeout_sec: float | None = None,
        vel: float | None = None,
        acc: float | None = None,
    ):
        self.movel_posx(pose6=pose6, label=label, enabled=enabled, timeout_sec=timeout_sec, vel=vel, acc=acc)

    def movej(
        self,
        joints6,
        label: str = "조인트 이동",
        enabled: bool = True,
        timeout_sec: float | None = None,
        vel: float | None = None,
        acc: float | None = None,
    ):
        self.movej_posj(joints6=joints6, label=label, enabled=enabled, timeout_sec=timeout_sec, vel=vel, acc=acc)

    def amovel(
        self,
        pose6,
        label: str = "카테시안 비동기 이동",
        enabled: bool = True,
        timeout_sec: float | None = None,
        vel: float | None = None,
        acc: float | None = None,
    ):
        self.amovel_posx(pose6=pose6, label=label, enabled=enabled, timeout_sec=timeout_sec, vel=vel, acc=acc)

    def amovej(
        self,
        joints6,
        label: str = "조인트 비동기 이동",
        enabled: bool = True,
        timeout_sec: float | None = None,
        vel: float | None = None,
        acc: float | None = None,
    ):
        self.amovej_posj(joints6=joints6, label=label, enabled=enabled, timeout_sec=timeout_sec, vel=vel, acc=acc)

    def gripper(self, distance_mm: float, label: str = "그리퍼 이동", enabled: bool = True):
        self._append(
            {
                "op": "backend_call",
                "label": str(label),
                "method": "send_gripper_move",
                "args": [float(distance_mm)],
                "kwargs": {},
                "enabled": bool(enabled),
            }
        )

    def wait_sec(self, seconds: float, label: str = "대기", enabled: bool = True):
        self._append(
            {
                "op": "wait_sec",
                "label": str(label),
                "seconds": float(seconds),
                "enabled": bool(enabled),
            }
        )

    def log_event(self, message: str, level: str = "info", enabled: bool = True):
        text = str(message or "").strip()
        if not text:
            return
        lvl = str(level or "info").strip().lower() or "info"
        self._append(
            {
                "op": "log_event",
                "label": text,
                "level": lvl,
                "enabled": bool(enabled),
            }
        )

    def motion_stop(self, label: str = "모션 정지", stop_mode: int = 2, enabled: bool = True):
        self._append(
            {
                "op": "backend_call",
                "label": str(label),
                "method": "send_motion_stop",
                "args": [int(stop_mode)],
                "kwargs": {},
                "enabled": bool(enabled),
            }
        )

    def emergency_stop(self, label: str = "정지", enabled: bool = True):
        self.motion_stop(label=label, stop_mode=2, enabled=enabled)

    def resolve_detection_target(
        self,
        *,
        target_key: str,
        ingredient_code: str,
        center_uv,
        depth_m: float,
        label: str = "비전 타겟 계산",
        extra_offset_xyz_mm=None,
        apply_menu_offset: bool = True,
        enabled: bool = True,
    ):
        self._append(
            {
                "op": "resolve_detection_target",
                "label": str(label),
                "target_key": str(target_key),
                "ingredient_code": str(ingredient_code),
                "target_uv": [float(center_uv[0]), float(center_uv[1])],
                "target_depth_m": float(depth_m),
                "apply_menu_offset": bool(apply_menu_offset),
                "extra_offset_xyz_mm": list(extra_offset_xyz_mm or [0.0, 0.0, 0.0]),
                "enabled": bool(enabled),
            }
        )

    def get_resolved_target_pose(self, *, target_key: str, approach_up_mm: float = 0.0, xyzabc=None, offset_xyz_mm=None, abc=None):
        payload = {
            "__resolved_target_key__": str(target_key),
            "approach_up_mm": float(approach_up_mm),
        }
        if xyzabc is not None:
            payload["xyzabc"] = list(xyzabc)
        if offset_xyz_mm is not None:
            payload["offset_xyz_mm"] = list(offset_xyz_mm)
        if abc is not None:
            payload["abc"] = list(abc)
        return payload

    def add_pose_offset_xyz(self, pose_payload, *, dx_mm: float = 0.0, dy_mm: float = 0.0, dz_mm: float = 0.0):
        if not isinstance(pose_payload, dict):
            raise ValueError("pose_payload는 get_resolved_target_pose() 반환 dict여야 합니다.")
        base = pose_payload.get("offset_xyz_mm", [0.0, 0.0, 0.0])
        try:
            bx = float(base[0]) if isinstance(base, (list, tuple)) and len(base) >= 1 else 0.0
            by = float(base[1]) if isinstance(base, (list, tuple)) and len(base) >= 2 else 0.0
            bz = float(base[2]) if isinstance(base, (list, tuple)) and len(base) >= 3 else 0.0
        except Exception:
            bx, by, bz = 0.0, 0.0, 0.0
        pose_payload["offset_xyz_mm"] = [
            float(bx) + float(dx_mm),
            float(by) + float(dy_mm),
            float(bz) + float(dz_mm),
        ]
        return pose_payload

    def movel_resolved_target(
        self,
        *,
        target_key: str,
        label: str = "비전 타겟 이동",
        approach_up_mm: float = 0.0,
        xyzabc=None,
        offset_xyz_mm=None,
        abc=None,
        enabled: bool = True,
        timeout_sec: float | None = None,
        vel: float | None = None,
        acc: float | None = None,
    ):
        step = {
            "op": "movel_resolved_target",
            "label": str(label),
            "target_key": str(target_key),
            "approach_up_mm": float(approach_up_mm),
            "enabled": bool(enabled),
        }
        if xyzabc is not None:
            step["xyzabc"] = list(xyzabc)
        if offset_xyz_mm is not None:
            step["offset_xyz_mm"] = list(offset_xyz_mm)
        if abc is not None:
            step["abc"] = list(abc)
        if timeout_sec is not None:
            step["timeout_sec"] = float(timeout_sec)
        if vel is not None:
            step["vel"] = float(vel)
        if acc is not None:
            step["acc"] = float(acc)
        self._append(step)

    def move_to_detection(
        self,
        *,
        ingredient_code: str,
        center_uv,
        depth_m: float,
        label: str,
        extra_offset_xyz_mm=None,
        approach_up_mm: float = 40.0,
        apply_menu_offset: bool = True,
        enabled: bool = True,
    ):
        key = f"legacy_{str(label)}_{str(ingredient_code)}"
        self.resolve_detection_target(
            target_key=key,
            ingredient_code=ingredient_code,
            center_uv=center_uv,
            depth_m=depth_m,
            label=f"{label} (타겟계산)",
            extra_offset_xyz_mm=extra_offset_xyz_mm,
            apply_menu_offset=apply_menu_offset,
            enabled=enabled,
        )
        pose = self.get_resolved_target_pose(target_key=key, approach_up_mm=approach_up_mm)
        self.movel_posx(pose, label=label, enabled=enabled)

    def wait_volume_target(
        self,
        target_volume_ml: float,
        label: str,
        timeout_sec: float = 20.0,
        poll_sec: float = 0.1,
        condition: str = "gte",
        compare_tolerance_ml: float = 0.5,
        enabled: bool = True,
    ):
        cond = str(condition or "gte").strip().lower()
        if cond not in ("gt", "gte", "eq", "lt", "lte"):
            cond = "gte"
        tol = max(0.0, float(compare_tolerance_ml))
        self._append(
            {
                "op": "wait_volume_target",
                "label": str(label),
                "target_volume_ml": float(target_volume_ml),
                "timeout_sec": float(timeout_sec),
                "poll_sec": float(poll_sec),
                "condition": cond,
                "compare_tolerance_ml": tol,
                "enabled": bool(enabled),
            }
        )

    def get_current_volume_ml(self, *, max_age_sec: float | None = None) -> float:
        raise RuntimeError(
            "PlannerSequenceApi는 실시간 vision2 현재용량 조회를 지원하지 않습니다. "
            "backend NativeRobotActionApi에서만 get_current_volume_ml()를 사용할 수 있습니다."
        )


def _safe_float(value: Any):
    try:
        return float(value)
    except Exception:
        return None


def _norm_code(value: Any):
    return str(value or "").strip().lower()


def _sanitize_gripper_close_mm(value: Any, default_mm: float = GRIPPER_CLOSE_MM_DEFAULT):
    v = _safe_float(value)
    if v is None or (not math.isfinite(v)):
        return float(default_mm)
    return max(0.0, min(float(GRIPPER_OPEN_MM_DEFAULT), float(v)))


def _clone_runtime_motion_config():
    return {
        "poses": {
            str(key): [float(v) for v in list(vals)[:6]]
            for key, vals in RUNTIME_POSE_DEFAULTS.items()
        },
        "offsets_xyz_mm": {
            str(key): [float(v) for v in list(vals)[:3]]
            for key, vals in RUNTIME_OFFSET_DEFAULTS_XYZ_MM.items()
        },
    }


def _as_pose6_or_none(raw: Any):
    if not isinstance(raw, (list, tuple)) or len(raw) < 6:
        return None
    out = []
    for idx in range(6):
        try:
            v = float(raw[idx])
        except Exception:
            return None
        if not math.isfinite(v):
            return None
        out.append(float(v))
    return out


def _as_xyz_or_none(raw: Any):
    if not isinstance(raw, (list, tuple)) or len(raw) < 3:
        return None
    out = []
    for idx in range(3):
        try:
            v = float(raw[idx])
        except Exception:
            return None
        if not math.isfinite(v):
            return None
        out.append(float(v))
    return out


def _canonical_runtime_pose_key(raw_key: Any):
    key = _norm_code(raw_key)
    if key in ("joint_pick_ready", "joint_pick_place_ready", "joint_service_ready"):
        return "service_ready_posj"
    if key == "joint_cheers":
        return "pour_start_cheers_posj"
    if key == "joint_contact":
        return "pour_contact_posj"
    if key == "joint_pour_horizontal":
        return "pour_horizontal_posj"
    if key == "joint_pour_diagonal":
        return "pour_diagonal_posj"
    if key == "joint_pour_vertical":
        return "pour_vertical_posj"
    if key in ("cup_pick_fallback_approach", "cup_pick_approach"):
        return "cup_pick_approach_posx"
    if key in ("cup_pick_fallback_pose", "cup_pick_pose"):
        return "cup_pick_pose_posx"
    if key in ("cup_pick_fallback_lift", "cup_pick_lift"):
        return "cup_pick_lift_posx"
    if key == "ingredient_pick_lift_out":
        return "pick_lift_out_posx"
    if key == "cup_delivery_approach":
        return "cup_delivery_approach_posx"
    if key == "cup_delivery_pose":
        return "cup_delivery_pose_posx"
    return key


def _canonical_runtime_offset_key(raw_key: Any):
    key = _norm_code(raw_key)
    if key == "ingredient_pick_approach":
        return "pick_approach_offset"
    if key == "ingredient_pick_grasp":
        return "pick_grasp_offset"
    if key == "ingredient_pick_lift":
        return "pick_lift_offset"
    if key == "ingredient_place":
        return "place_offset"
    if key == "ingredient_retreat":
        return "retreat_offset"
    return key


def _apply_runtime_motion_override(runtime_cfg: dict, raw: Any):
    if not isinstance(runtime_cfg, dict) or not isinstance(raw, dict):
        return
    poses_dst = runtime_cfg.get("poses", {})
    offsets_dst = runtime_cfg.get("offsets_xyz_mm", {})
    if not isinstance(poses_dst, dict) or not isinstance(offsets_dst, dict):
        return

    poses_raw = raw.get("poses")
    if isinstance(poses_raw, dict):
        explicit_pose_keys = set()
        legacy_ready_posj = None
        for key, value in poses_raw.items():
            norm_key = _canonical_runtime_pose_key(key)
            if not norm_key:
                continue
            for dst_key in list(poses_dst.keys()):
                if _norm_code(dst_key) != norm_key:
                    continue
                parsed = _as_pose6_or_none(value)
                if parsed is not None:
                    poses_dst[dst_key] = list(parsed)
                    explicit_pose_keys.add(norm_key)
                    if _norm_code(key) in ("joint_pick_ready", "joint_pick_place_ready", "joint_service_ready"):
                        legacy_ready_posj = list(parsed)
                break
        if legacy_ready_posj is not None:
            for inherit_key in ("cup_pick_ready_posj", "cup_delivery_ready_posj"):
                if inherit_key in explicit_pose_keys:
                    continue
                for dst_key in list(poses_dst.keys()):
                    if _norm_code(dst_key) != inherit_key:
                        continue
                    poses_dst[dst_key] = list(legacy_ready_posj)
                    break

    offsets_raw = raw.get("offsets_xyz_mm")
    if isinstance(offsets_raw, dict):
        for key, value in offsets_raw.items():
            norm_key = _canonical_runtime_offset_key(key)
            if not norm_key:
                continue
            for dst_key in list(offsets_dst.keys()):
                if _norm_code(dst_key) != norm_key:
                    continue
                parsed = _as_xyz_or_none(value)
                if parsed is not None:
                    offsets_dst[dst_key] = list(parsed)
                break


def _resolve_runtime_motion_config(context: dict):
    runtime_cfg = _clone_runtime_motion_config()

    try:
        if os.path.isfile(ROBOT_ACTION_POSE_CONFIG_PATH):
            with open(ROBOT_ACTION_POSE_CONFIG_PATH, "r", encoding="utf-8") as fp:
                loaded = json.load(fp)
            _apply_runtime_motion_override(runtime_cfg, loaded)
    except Exception:
        pass

    ctx = dict(context or {})
    _apply_runtime_motion_override(runtime_cfg, ctx.get("robot_action_pose_config"))
    return runtime_cfg


def _runtime_pose6(runtime_cfg: dict | None, key: str, fallback_pose6):
    fallback = _as_pose6_or_none(fallback_pose6)
    if fallback is None:
        fallback = [0.0] * 6
    if isinstance(runtime_cfg, dict):
        poses = runtime_cfg.get("poses", {})
        if isinstance(poses, dict):
            parsed = _as_pose6_or_none(poses.get(str(key)))
            if parsed is not None:
                return parsed
    return list(fallback)


def _runtime_offset_xyz(runtime_cfg: dict | None, key: str, fallback_xyz=(0.0, 0.0, 0.0)):
    fallback = _as_xyz_or_none(fallback_xyz)
    if fallback is None:
        fallback = [0.0, 0.0, 0.0]
    if isinstance(runtime_cfg, dict):
        offsets = runtime_cfg.get("offsets_xyz_mm", {})
        if isinstance(offsets, dict):
            parsed = _as_xyz_or_none(offsets.get(str(key)))
            if parsed is not None:
                return parsed
    return list(fallback)


def _supports_live_volume_feedback(api: PlannerSequenceApi):
    return hasattr(api, "backend") and hasattr(api, "get_current_volume_ml")


def _emit_sequence_log(api: PlannerSequenceApi, text: str, level: str = "info", enabled: bool = True):
    message = str(text or "").strip()
    if not message:
        return
    log_event = getattr(api, "log_event", None)
    if callable(log_event):
        log_event(message, level=level, enabled=enabled)
        return
    append_log = getattr(api, "_append_log", None)
    if callable(append_log):
        append_log(message)


def _is_live_execution_mode(api: PlannerSequenceApi):
    if not _supports_live_volume_feedback(api):
        return False
    try:
        return bool(getattr(api, "execute_enabled", True))
    except Exception:
        return True


# ---------------------------------------------------------------------------
# 3) 데이터 준비(주문/비전)
# ---------------------------------------------------------------------------


def _extract_order_result(context: dict):
    ctx = dict(context or {})
    order_result = dict(ctx.get("order_result", {}) or {})
    status = str(order_result.get("status", "")).strip().lower()
    if status != "success":
        return None, {
            "ok": False,
            "status": "order_not_success",
            "message": "주문 결과 status가 success가 아닙니다.",
        }
    return order_result, None


def _extract_vision1_detections(context: dict):
    ctx = dict(context or {})
    vision1_meta = dict(ctx.get("vision1_meta", {}) or {})
    detections = vision1_meta.get("detections", [])
    if not isinstance(detections, list):
        detections = []
    return detections


def _extract_menu_gripper_close_map(context: dict):
    ctx = dict(context or {})
    raw = ctx.get("menu_offsets", {})
    if not isinstance(raw, dict):
        return {}
    source = raw.get("menus", raw) if isinstance(raw.get("menus", None), dict) else raw
    out = {}
    for key, payload in source.items():
        code = _norm_code(key)
        if (not code) or (not isinstance(payload, dict)):
            continue
        try:
            value = float(payload.get("gripper_close_mm"))
        except Exception:
            continue
        if (not math.isfinite(value)) or value < 0.0:
            continue
        out[code] = float(value)
    return out


def _match_class_alias(class_name: str, aliases):
    name = _norm_code(class_name)
    alias_norm = tuple(_norm_code(v) for v in tuple(aliases or ()))
    if not name:
        return False
    if name in alias_norm:
        return True
    return any(alias and (alias in name or name in alias) for alias in alias_norm)


def _parse_detection(det):
    if not isinstance(det, dict):
        return None
    class_name = _norm_code(det.get("class_name"))
    center_uv = det.get("center_uv")
    if not isinstance(center_uv, (list, tuple)) or len(center_uv) < 2:
        bbox = det.get("bbox_xyxy")
        if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
            try:
                x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
                center_uv = [(x1 + x2) * 0.5, (y1 + y2) * 0.5]
            except Exception:
                center_uv = None
    if not isinstance(center_uv, (list, tuple)) or len(center_uv) < 2:
        return None

    depth_m = _safe_float(det.get("depth_m"))
    if depth_m is None or depth_m <= 0.0 or (not math.isfinite(depth_m)):
        return None

    conf = _safe_float(det.get("confidence"))
    score = float(conf if conf is not None and math.isfinite(conf) else 0.0)
    return {
        "class_name": class_name,
        "confidence": score,
        "center_uv": [float(center_uv[0]), float(center_uv[1])],
        "depth_m": float(depth_m),
    }


def _pick_detection_for_ingredient(detections, ingredient_code: str):
    aliases = INGREDIENT_CLASS_ALIASES.get(_norm_code(ingredient_code), (ingredient_code,))
    best = None
    best_score = -1.0
    for det in detections if isinstance(detections, list) else []:
        parsed = _parse_detection(det)
        if parsed is None:
            continue
        if not _match_class_alias(parsed.get("class_name", ""), aliases):
            continue
        score = float(parsed.get("confidence", 0.0))
        if score >= best_score:
            best = parsed
            best_score = score
    return best


def _pick_cup_detection(detections):
    best = None
    best_score = -1.0
    for det in detections if isinstance(detections, list) else []:
        parsed = _parse_detection(det)
        if parsed is None:
            continue
        if not _match_class_alias(parsed.get("class_name", ""), CUP_CLASS_ALIASES):
            continue
        score = float(parsed.get("confidence", 0.0))
        if score >= best_score:
            best = parsed
            best_score = score
    return best


def _resolve_recipe_in_menu_order(order_result: dict):
    selected_menu = _norm_code(order_result.get("selected_menu"))
    raw_recipe = order_result.get("recipe", {})
    recipe = dict(raw_recipe) if isinstance(raw_recipe, dict) else {}

    ordered_codes = list(MENU_ORDER.get(selected_menu, []))
    if not ordered_codes and recipe:
        ordered_codes = [_norm_code(k) for k in recipe.keys()]
    if not ordered_codes:
        return [], selected_menu, "주문에서 메뉴/레시피를 확인할 수 없습니다."

    resolved = []
    seen = set()
    for code in ordered_codes:
        code = _norm_code(code)
        if not code:
            continue
        amount = _safe_float(recipe.get(code))
        if amount is None:
            amount = _safe_float(MENU_DEFAULT_RECIPE.get(selected_menu, {}).get(code))
        if amount is None or amount <= 0.0:
            return [], selected_menu, f"레시피 누락: 메뉴={selected_menu}, 재료={code}"
        resolved.append((code, float(amount)))
        seen.add(code)

    for key, value in recipe.items():
        code = _norm_code(key)
        if (not code) or code in seen:
            continue
        amount = _safe_float(value)
        if amount is None or amount <= 0.0:
            continue
        resolved.append((code, float(amount)))
        seen.add(code)
    return resolved, selected_menu, ""


def _resolve_targets_from_recipe(recipe_items, detections, gripper_close_map=None):
    picked_targets = []
    missing_ingredients = []
    resolved_targets = []
    grip_map = dict(gripper_close_map or {})

    cumulative_target_ml = 0.0
    for ingredient_code, amount_ml in recipe_items:
        det = _pick_detection_for_ingredient(detections, ingredient_code)
        if det is None:
            missing_ingredients.append(ingredient_code)
            continue

        cumulative_target_ml += float(amount_ml)
        row = {
            "ingredient_code": str(ingredient_code),
            "amount_ml": float(amount_ml),
            "target_volume_ml": float(cumulative_target_ml),
            "gripper_close_mm": _sanitize_gripper_close_mm(grip_map.get(_norm_code(ingredient_code), GRIPPER_CLOSE_MM_DEFAULT)),
            "detection": dict(det),
        }
        resolved_targets.append(row)
        picked_targets.append(dict(row))

    return picked_targets, missing_ingredients, resolved_targets


# ---------------------------------------------------------------------------
# 4) 런타임 피드백 유틸
# ---------------------------------------------------------------------------


def _read_current_volume_safe(api: PlannerSequenceApi, *, max_age_sec: float = 0.8):
    try:
        value = float(api.get_current_volume_ml(max_age_sec=max_age_sec))
    except Exception as exc:
        return None, str(exc)
    if not math.isfinite(value):
        return None, "현재 용량값이 유한하지 않습니다."
    return float(value), "ok"


def _snapback_to_cheers_path(
    api: PlannerSequenceApi,
    *,
    ingredient_code: str,
    reverse_path: list[tuple[str, list[float]]],
    cheers_posj,
):
    movesj_fn = getattr(api, "movesj_posj", None)
    if not callable(movesj_fn):
        raise RuntimeError(f"[{ingredient_code}] 스냅복귀 실패: movesj_posj 미지원")
    path = [list(joints) for _name, joints in list(reverse_path)]
    path.append(list(cheers_posj))
    movesj_fn(path, label=f"[{ingredient_code}] 스냅복귀 movesj", vel=120.0, acc=120.0)


def _run_movesx_path_with_volume_trigger(
    api: PlannerSequenceApi,
    *,
    ingredient_code: str,
    path_name: str,
    joints_path,
    target_volume_ml: float,
    vel: float,
    acc: float,
    poll_sec: float = LIVE_POUR_VOLUME_POLL_SEC,
):
    movesx_from_posj = getattr(api, "movesx_from_posj_path", None)
    if not callable(movesx_from_posj):
        raise RuntimeError(f"[{ingredient_code}] {path_name} 실패: movesx_from_posj_path 미지원")
    backend = getattr(api, "backend", None)
    if backend is None or (not hasattr(backend, "send_motion_stop_async")):
        raise RuntimeError(f"[{ingredient_code}] {path_name} 실패: motion_stop 미지원")
    if not isinstance(joints_path, (list, tuple)) or len(joints_path) < 2:
        raise RuntimeError(f"[{ingredient_code}] {path_name} 실패: movesx 경로(2점 이상) 필요")

    stop_watch = threading.Event()
    reached = threading.Event()
    state = {"last_volume": None, "stop_error": ""}

    def _watch_volume_and_trigger():
        while not stop_watch.is_set():
            volume_now, _msg = _read_current_volume_safe(api, max_age_sec=0.8)
            if volume_now is not None:
                state["last_volume"] = float(volume_now)
                if float(state["last_volume"]) >= float(target_volume_ml):
                    ok_stop, msg_stop = backend.send_motion_stop_async(stop_mode=2)
                    if not ok_stop:
                        state["stop_error"] = str(msg_stop or "정지 실패")
                    reached.set()
                    break
            time.sleep(max(0.01, float(poll_sec)))

    watcher = threading.Thread(
        target=_watch_volume_and_trigger,
        daemon=True,
        name=f"live-pour-watch-{ingredient_code}-{path_name}",
    )
    watcher.start()

    move_exc = None
    try:
        movesx_from_posj(
            [list(v) for v in list(joints_path)],
            label=f"[{ingredient_code}] {path_name}",
            vel=float(vel),
            acc=float(acc),
        )
    except Exception as exc:
        move_exc = exc
    finally:
        stop_watch.set()
        try:
            watcher.join(timeout=0.05)
        except Exception:
            pass

    stop_err = str(state.get("stop_error", "") or "").strip()
    if stop_err:
        raise RuntimeError(f"[{ingredient_code}] {path_name} 실패: 목표용량 도달 정지 실패({stop_err})")
    if move_exc is not None and (not reached.is_set()):
        raise move_exc
    return bool(reached.is_set()), state.get("last_volume")


def _estimate_reached_stage_index_from_current_posj(api: PlannerSequenceApi, joints_path):
    backend = getattr(api, "backend", None)
    if backend is None or (not hasattr(backend, "get_position_snapshot")):
        return max(0, len(list(joints_path or [])) - 1)
    try:
        snapshot, _seen_at = backend.get_position_snapshot()
    except Exception:
        snapshot = None
    if not isinstance(snapshot, (list, tuple)) or len(snapshot) < 1:
        return max(0, len(list(joints_path or [])) - 1)
    posj_now = snapshot[0]
    if not isinstance(posj_now, (list, tuple)) or len(posj_now) < 6:
        return max(0, len(list(joints_path or [])) - 1)
    try:
        now = [float(v) for v in list(posj_now)[:6]]
    except Exception:
        return max(0, len(list(joints_path or [])) - 1)
    best_idx = 0
    best_score = float("inf")
    for idx, joints in enumerate(list(joints_path or [])):
        if not isinstance(joints, (list, tuple)) or len(joints) < 6:
            continue
        try:
            vals = [float(v) for v in list(joints)[:6]]
        except Exception:
            continue
        score = sum(abs(now[i] - vals[i]) for i in range(6))
        if score < best_score:
            best_score = float(score)
            best_idx = int(idx)
    return int(best_idx)


def _execute_live_volume_feedback_pour_sequence(
    api: PlannerSequenceApi,
    *,
    ingredient_code: str,
    target_volume_ml: float,
    runtime_cfg: dict | None = None,
):
    if not _supports_live_volume_feedback(api):
        raise RuntimeError(f"[{ingredient_code}] 실시간 용량 피드백 따르기 실행 실패: backend 미지원")

    target_ml = float(target_volume_ml)
    _emit_sequence_log(api, f"[{ingredient_code}] 실시간 따르기 시작(target={target_ml:.1f}ml)")
    pour_start_cheers_posj = _runtime_pose6(runtime_cfg, "pour_start_cheers_posj", POUR_START_CHEERS_POSJ)
    pour_contact_posj = _runtime_pose6(runtime_cfg, "pour_contact_posj", POUR_CONTACT_POSJ)
    pour_horizontal_posj = _runtime_pose6(runtime_cfg, "pour_horizontal_posj", POUR_HORIZONTAL_POSJ)
    pour_diagonal_posj = _runtime_pose6(runtime_cfg, "pour_diagonal_posj", POUR_DIAGONAL_POSJ)
    pour_vertical_posj = _runtime_pose6(runtime_cfg, "pour_vertical_posj", POUR_VERTICAL_POSJ)

    # Setup trajectories (원본 형태 유지)
    p0 = list(pour_start_cheers_posj)
    p1 = list(pour_start_cheers_posj)
    p2 = list(pour_contact_posj)
    p3 = list(pour_horizontal_posj)
    p4 = list(pour_diagonal_posj)
    p5 = list(pour_vertical_posj)
    approach_path = [p0, p1, p2]
    pour_path = [p2, p3, p4, p5]
    forward_path = [
        ("contact", p2),
        ("pour_horizontal", p3),
        ("pour_diagonal", p4),
        ("pour_vertical", p5),
    ]
    reached_idx = len(forward_path) - 1
    last_volume = None
    reached = False

    reached_ap, volume_ap = _run_movesx_path_with_volume_trigger(
        api,
        ingredient_code=ingredient_code,
        path_name="접근 movesx(approach_path)",
        joints_path=approach_path,
        target_volume_ml=target_ml,
        vel=60.0,
        acc=60.0,
        poll_sec=LIVE_POUR_VOLUME_POLL_SEC,
    )
    if volume_ap is not None:
        last_volume = float(volume_ap)
    if reached_ap:
        reached = True
        reached_idx = 0
        _emit_sequence_log(api, f"[{ingredient_code}] 목표용량 도달 감지(stage=contact)")
    else:
        reached_pour, volume_pour = _run_movesx_path_with_volume_trigger(
            api,
            ingredient_code=ingredient_code,
            path_name="따르기 movesx(pour_path)",
            joints_path=pour_path,
            target_volume_ml=target_ml,
            vel=60.0,
            acc=60.0,
            poll_sec=LIVE_POUR_VOLUME_POLL_SEC,
        )
        if volume_pour is not None:
            last_volume = float(volume_pour)
        if reached_pour:
            reached = True
            reached_idx = _estimate_reached_stage_index_from_current_posj(api, pour_path)
            stage_name = str(forward_path[min(max(0, reached_idx), len(forward_path) - 1)][0])
            _emit_sequence_log(api, f"[{ingredient_code}] 목표용량 도달 감지(stage={stage_name})")

    if not reached:
        reached_idx = len(forward_path) - 1
        deadline = time.monotonic() + 5.0
        while time.monotonic() <= deadline:
            volume_now, _msg = _read_current_volume_safe(api, max_age_sec=0.8)
            if volume_now is not None:
                last_volume = float(volume_now)
                if float(last_volume) >= float(target_ml):
                    reached = True
                    break
            time.sleep(LIVE_POUR_FINAL_WAIT_POLL_SEC)
        if not reached:
            try:
                api.motion_stop(label=f"[{ingredient_code}] 목표 미도달 안전정지")
            except Exception:
                pass
            _emit_sequence_log(api, f"[{ingredient_code}] 목표용량 미도달 안전복귀 시작", level="warning")
            reverse_path = list(reversed(forward_path[: reached_idx + 1]))
            _snapback_to_cheers_path(
                api,
                ingredient_code=ingredient_code,
                reverse_path=reverse_path,
                cheers_posj=pour_start_cheers_posj,
            )
            if last_volume is None:
                _emit_sequence_log(api, f"[{ingredient_code}] 목표용량 미도달(현재용량 읽기 실패)", level="error")
                raise RuntimeError(f"[{ingredient_code}] 목표용량 미도달(현재용량 읽기 실패 / 목표 {target_ml:.1f}ml)")
            _emit_sequence_log(
                api,
                f"[{ingredient_code}] 목표용량 미도달({last_volume:.1f}/{target_ml:.1f}ml)",
                level="error",
            )
            raise RuntimeError(f"[{ingredient_code}] 목표용량 미도달({last_volume:.1f}/{target_ml:.1f}ml)")
    reverse_path = list(reversed(forward_path[: reached_idx + 1]))
    _snapback_to_cheers_path(
        api,
        ingredient_code=ingredient_code,
        reverse_path=reverse_path,
        cheers_posj=pour_start_cheers_posj,
    )
    _emit_sequence_log(api, f"[{ingredient_code}] 실시간 따르기 종료")


# ---------------------------------------------------------------------------
# 5) 모션 시퀀스 구성
# ---------------------------------------------------------------------------


def _resolved_target_pose(
    api: PlannerSequenceApi,
    *,
    target_key: str,
    dx_mm: float = 0.0,
    dy_mm: float = 0.0,
    dz_mm: float = 0.0,
    approach_up_mm: float = 0.0,
    abc=None,
):
    payload = api.get_resolved_target_pose(
        target_key=str(target_key),
        approach_up_mm=float(approach_up_mm),
        abc=abc,
    )
    if abs(float(dx_mm)) > 1e-9 or abs(float(dy_mm)) > 1e-9 or abs(float(dz_mm)) > 1e-9:
        api.add_pose_offset_xyz(payload, dx_mm=float(dx_mm), dy_mm=float(dy_mm), dz_mm=float(dz_mm))
    return payload


def _append_start_sequence(api: PlannerSequenceApi, runtime_cfg: dict | None = None):
    service_ready_posj = _runtime_pose6(runtime_cfg, "service_ready_posj", SERVICE_READY_POSJ)
    _emit_sequence_log(api, "[system] 제조 시퀀스 시작")
    _emit_sequence_log(api, "[system] 초기화 구간 시작")
    api.set_robot_mode(mode=1, label="[system] 로봇 오토모드 전환")
    api.move_home(label="[system] 초기 홈 이동")
    api.movej_posj(service_ready_posj, label="[system] 공통 준비자세")
    _emit_sequence_log(api, "[system] 초기화 구간 종료")


def _append_ingredient_sequence(api: PlannerSequenceApi, row: dict, seq_index: int, runtime_cfg: dict | None = None):
    ingredient_code = str(row["ingredient_code"])
    center_uv = row["detection"]["center_uv"]
    depth_m = float(row["detection"]["depth_m"])
    target_volume_ml = float(row["target_volume_ml"])
    target_volume_text = f"{target_volume_ml:.1f}ml"
    gripper_close_mm = _sanitize_gripper_close_mm(row.get("gripper_close_mm", GRIPPER_CLOSE_MM_DEFAULT))
    target_key = f"ingredient_{int(seq_index)}_{ingredient_code}_target"
    service_ready_posj = _runtime_pose6(runtime_cfg, "service_ready_posj", SERVICE_READY_POSJ)
    pour_start_cheers_posj = _runtime_pose6(runtime_cfg, "pour_start_cheers_posj", POUR_START_CHEERS_POSJ)
    pour_contact_posj = _runtime_pose6(runtime_cfg, "pour_contact_posj", POUR_CONTACT_POSJ)
    pour_horizontal_posj = _runtime_pose6(runtime_cfg, "pour_horizontal_posj", POUR_HORIZONTAL_POSJ)
    pour_diagonal_posj = _runtime_pose6(runtime_cfg, "pour_diagonal_posj", POUR_DIAGONAL_POSJ)
    pour_vertical_posj = _runtime_pose6(runtime_cfg, "pour_vertical_posj", POUR_VERTICAL_POSJ)
    pick_approach_offset = _runtime_offset_xyz(runtime_cfg, "pick_approach_offset", [0.0, -50.0, 0.0])
    pick_grasp_offset = _runtime_offset_xyz(runtime_cfg, "pick_grasp_offset", [0.0, 0.0, 0.0])
    pick_lift_offset = _runtime_offset_xyz(runtime_cfg, "pick_lift_offset", [0.0, 0.0, 100.0])
    pick_lift_out_posx = _runtime_pose6(runtime_cfg, "pick_lift_out_posx", PICK_LIFT_OUT_POSX)
    place_offset = _runtime_offset_xyz(runtime_cfg, "place_offset", [0.0, 0.0, 0.0])
    retreat_offset = _runtime_offset_xyz(runtime_cfg, "retreat_offset", [-20.0, -50.0, 0.0])
    _emit_sequence_log(api, f"[{ingredient_code}] 재료 시퀀스 시작(누적목표={target_volume_text})")

    # --- [1] PICK 병 집기 구역 ---
    _emit_sequence_log(api, f"[{ingredient_code}] [1] PICK 시작")
    api.gripper(GRIPPER_OPEN_MM_DEFAULT, label=f"[{ingredient_code}] 그리퍼 열림")
    api.wait_sec(1.0, label=f"[{ingredient_code}] 그리퍼 열림 대기")
    api.movej_posj(service_ready_posj, label=f"[{ingredient_code}] 병 집기 준비")

    api.resolve_detection_target(
        target_key=target_key,
        ingredient_code=ingredient_code,
        center_uv=center_uv,
        depth_m=depth_m,
        label=f"[{ingredient_code}] 병 타겟 계산",
        extra_offset_xyz_mm=[0.0, 0.0, 0.0],
        apply_menu_offset=True,
    )

    # 병 집기 표준 시퀀스
    # target_1(접근) -> target_2(파지)
    pick_approach_posx = _resolved_target_pose(
        api,
        target_key=target_key,
        dx_mm=float(pick_approach_offset[0]),
        dy_mm=float(pick_approach_offset[1]),
        dz_mm=float(pick_approach_offset[2]),
    )
    pick_grasp_posx = _resolved_target_pose(
        api,
        target_key=target_key,
        dx_mm=float(pick_grasp_offset[0]),
        dy_mm=float(pick_grasp_offset[1]),
        dz_mm=float(pick_grasp_offset[2]),
    )
    pick_lift_posx = _resolved_target_pose(
        api,
        target_key=target_key,
        dx_mm=float(pick_lift_offset[0]),
        dy_mm=float(pick_lift_offset[1]),
        dz_mm=float(pick_lift_offset[2]),
    )
    api.movel_posx(pick_approach_posx, label=f"[{ingredient_code}] 병 접근(target_1)")
    api.movel_posx(pick_grasp_posx, label=f"[{ingredient_code}] 병 파지(target_2)", vel=40.0, acc=40.0)
    api.gripper(gripper_close_mm, label=f"[{ingredient_code}] 병 파지")
    api.wait_sec(3.0, label=f"[{ingredient_code}] 병 파지 대기")
    api.movel_posx(pick_lift_posx, label=f"[{ingredient_code}] 병 파지 후 업(target_3)", vel=40.0, acc=40.0)
    api.movel_posx(pick_lift_out_posx, label=f"[{ingredient_code}] 병 파지 후 업 이탈(target_4)", vel=40.0, acc=40.0)
    #api.movej_posj(service_ready_posj, label=f"[{ingredient_code}] 병 파지 후 준비자세")
    api.move_home(label=f"[{ingredient_code}] 병 파지 후 홈 이동")
    _emit_sequence_log(api, f"[{ingredient_code}] [1] PICK 종료")

    # --- [2] POUR 따르기 구역 ---
    # 표준 따르기 시퀀스
    # CHEERS -> CONTACT -> HORIZONTAL -> DIAGONAL -> VERTICAL -> STOP -> SNAP(역순복귀) -> CHEERS
    _emit_sequence_log(api, f"[{ingredient_code}] [2] POUR 시작(target={target_volume_text})")
    api.movej_posj(pour_start_cheers_posj, label=f"[{ingredient_code}] 따르기 시작 cheers", vel=60.0, acc=60.0)

    if _is_live_execution_mode(api):
        _execute_live_volume_feedback_pour_sequence(
            api,
            ingredient_code=ingredient_code,
            target_volume_ml=target_volume_ml,
            runtime_cfg=runtime_cfg,
        )
    else:
        api.movej_posj(pour_contact_posj, label=f"[{ingredient_code}] 따르기 contact", vel=60.0, acc=60.0)
        api.movej_posj(pour_horizontal_posj, label=f"[{ingredient_code}] 따르기 horizontal", vel=60.0, acc=60.0)
        api.movej_posj(pour_diagonal_posj, label=f"[{ingredient_code}] 따르기 diagonal", vel=60.0, acc=60.0)
        api.movej_posj(pour_vertical_posj, label=f"[{ingredient_code}] 따르기 vertical", vel=60.0, acc=60.0)

        api.wait_volume_target(
            target_volume_ml=target_volume_ml,
            label=f"[{ingredient_code}] 용량도달 대기({target_volume_text})",
            timeout_sec=40.0,
            poll_sec=0.02,
            condition="gte",
            compare_tolerance_ml=0.5,
        )
        api.motion_stop(label=f"[{ingredient_code}] 목표용량 도달 정지")
        api.movej_posj(pour_diagonal_posj, label=f"[{ingredient_code}] 스냅복귀 diagonal", vel=120.0, acc=120.0)
        api.movej_posj(pour_horizontal_posj, label=f"[{ingredient_code}] 스냅복귀 horizontal", vel=120.0, acc=120.0)
        api.movej_posj(pour_contact_posj, label=f"[{ingredient_code}] 스냅복귀 contact", vel=120.0, acc=120.0)
        api.movej_posj(pour_start_cheers_posj, label=f"[{ingredient_code}] 스냅복귀 cheers", vel=120.0, acc=120.0)
    _emit_sequence_log(api, f"[{ingredient_code}] [2] POUR 종료")

    # --- [3] RETURN 병 원위치 복귀 구역 ---
    # 병 원위치 복귀 표준 시퀀스
    # target_2(안착) -> target_1(이탈)
    _emit_sequence_log(api, f"[{ingredient_code}] [3] RETURN 시작")
    place_posx = _resolved_target_pose(
        api,
        target_key=target_key,
        dx_mm=float(place_offset[0]),
        dy_mm=float(place_offset[1]),
        dz_mm=float(place_offset[2]),
    )
    retreat_posx = _resolved_target_pose(
        api,
        target_key=target_key,
        dx_mm=float(retreat_offset[0]),
        dy_mm=float(retreat_offset[1]),
        dz_mm=float(retreat_offset[2]),
    )

    # 픽 거꾸로 시퀸스
    api.movej_posj(service_ready_posj, label=f"[{ingredient_code}] 원위치 복귀 준비")
    api.movel_posx(pick_lift_out_posx, label=f"[{ingredient_code}] 병 파지 후 업 이탈(target_4)", vel=40.0, acc=40.0)
    api.movel_posx(pick_lift_posx, label=f"[{ingredient_code}] 병 파지 후 업(target_3)", vel=40.0, acc=40.0)
    api.movel_posx(pick_grasp_posx, label=f"[{ingredient_code}] 병 파지(target_2)", vel=40.0, acc=40.0)

    #api.movel_posx(place_posx, label=f"[{ingredient_code}] 원위치 안착(target_2)", vel=40.0, acc=40.0)
    api.gripper(GRIPPER_OPEN_MM_DEFAULT, label=f"[{ingredient_code}] 병 놓기")
    api.wait_sec(2.0, label=f"[{ingredient_code}] 릴리즈 대기")
    #api.movel_posx(retreat_posx, label=f"[{ingredient_code}] 원위치 이탈(target_1)", vel=40.0, acc=40.0)
    api.movel_posx(pick_approach_posx, label=f"[{ingredient_code}] 병 접근(target_1)")
    api.movej_posj(service_ready_posj, label=f"[{ingredient_code}] 다음 재료 준비")
    _emit_sequence_log(api, f"[{ingredient_code}] [3] RETURN 종료")
    _emit_sequence_log(api, f"[{ingredient_code}] 재료 시퀀스 종료")
    #api.move_home(label=f"[{ingredient_code}] 원위치 복귀 후 홈 이동")


def _append_finish_sequence(
    api: PlannerSequenceApi,
    *,
    cup_detection: dict | None,
    cup_gripper_close_mm: float,
    cup_gripper_open_mm: float,
    runtime_cfg: dict | None = None,
):
    _ = cup_detection
    _emit_sequence_log(api, "[glass] 완성컵 처리 시퀀스 시작")
    ''''
    close_mm = _sanitize_gripper_close_mm(cup_gripper_close_mm)
    open_mm = _sanitize_gripper_close_mm(cup_gripper_open_mm, default_mm=GRIPPER_OPEN_MM_DEFAULT)
    cup_pick_ready_posj = _runtime_pose6(runtime_cfg, "cup_pick_ready_posj", CUP_PICK_READY_POSJ)
    cup_pick_approach_posx = _runtime_pose6(
        runtime_cfg,
        "cup_pick_approach_posx",
        CUP_PICK_APPROACH_POSX,
    )
    cup_pick_pose_posx = _runtime_pose6(runtime_cfg, "cup_pick_pose_posx", CUP_PICK_POSE_POSX)
    cup_pick_lift_posx = _runtime_pose6(runtime_cfg, "cup_pick_lift_posx", CUP_PICK_LIFT_POSX)
    cup_delivery_ready_posj = _runtime_pose6(runtime_cfg, "cup_delivery_ready_posj", CUP_DELIVERY_READY_POSJ)
    cup_delivery_approach_posx = _runtime_pose6(runtime_cfg, "cup_delivery_approach_posx", CUP_DELIVERY_APPROACH_POSX)
    cup_delivery_pose_posx = _runtime_pose6(runtime_cfg, "cup_delivery_pose_posx", CUP_DELIVERY_POSE_POSX)

    _emit_sequence_log(api, "[glass] [4] CUP PICK 시작")
    api.gripper(open_mm, label="[glass] 컵 집기 전 그리퍼 열림")
    api.wait_sec(0.8, label="[glass] 컵 집기 전 대기")
    api.movej_posj(cup_pick_ready_posj, label="[glass] 컵 집기 준비자세")
    api.movel_posx(cup_pick_approach_posx, label="[glass] 완성컵 집기 접근(고정)")
    api.movel_posx(cup_pick_pose_posx, label="[glass] 완성컵 집기 위치(고정)", vel=15.0, acc=15.0)
    api.gripper(close_mm, label="[glass] 완성컵 파지(고정)")
    api.wait_sec(1.0, label="[glass] 완성컵 파지 대기")
    api.movel_posx(cup_pick_lift_posx, label="[glass] 완성컵 리프트(고정)")
    _emit_sequence_log(api, "[glass] [4] CUP PICK 종료")

    _emit_sequence_log(api, "[glass] [5] DELIVERY 시작")
    api.movej_posj(cup_delivery_ready_posj, label="[glass] 전달 준비자세")
    api.movel_posx(cup_delivery_approach_posx, label="[glass] 전달 위치 접근")
    api.movel_posx(cup_delivery_pose_posx, label="[glass] 전달 위치 안착")
    api.gripper(open_mm, label="[glass] 완성컵 릴리즈")
    api.wait_sec(1.0, label="[glass] 릴리즈 대기")
    api.movel_posx(cup_delivery_approach_posx, label="[glass] 전달 위치 이탈")
    _emit_sequence_log(api, "[glass] [5] DELIVERY 종료")

    _emit_sequence_log(api, "[system] 제조 시퀀스 종료")
    api.move_home(label="[system] 제조 종료 홈 복귀")
    '''

# ---------------------------------------------------------------------------
# 6) 진입점
# ---------------------------------------------------------------------------


def run_robot_action(context: dict, api: PlannerSequenceApi | None = None):
    order_result, err_out = _extract_order_result(context)
    if err_out is not None:
        return dict(err_out)

    recipe_items, selected_menu, recipe_err = _resolve_recipe_in_menu_order(order_result)
    if recipe_err:
        return {"ok": False, "status": "recipe_invalid", "message": recipe_err}

    detections = _extract_vision1_detections(context)
    gripper_close_map = _extract_menu_gripper_close_map(context)
    sequence_api = api if api is not None else PlannerSequenceApi()
    runtime_motion_cfg = _resolve_runtime_motion_config(context)

    picked_targets, missing_ingredients, resolved_targets = _resolve_targets_from_recipe(
        recipe_items,
        detections,
        gripper_close_map=gripper_close_map,
    )

    cup_detection = None
    _emit_sequence_log(
        sequence_api,
        f"[system] 레시피 확인 완료(menu={selected_menu or '-'}, ingredients={len(recipe_items)})",
    )

    if missing_ingredients:
        _emit_sequence_log(
            sequence_api,
            f"[system] 재료 감지 실패: {', '.join(missing_ingredients)}",
            level="error",
        )
        return {
            "ok": False,
            "status": "vision_target_missing",
            "message": f"vision1에서 레시피 재료를 찾지 못했습니다: {', '.join(missing_ingredients)}",
            "missing_ingredients": list(missing_ingredients),
            "plan": {
                "selected_menu": selected_menu,
                "recipe_items": [{"code": c, "amount_ml": float(a)} for c, a in recipe_items],
                "picked_targets": picked_targets,
                "cup_target": cup_detection,
                "runtime_motion_cfg": runtime_motion_cfg,
                "sequence_steps": sequence_api.sequence_steps,
            },
        }

    _append_start_sequence(sequence_api, runtime_cfg=runtime_motion_cfg)
    for idx, row in enumerate(resolved_targets, start=1):
        _append_ingredient_sequence(sequence_api, row, seq_index=idx, runtime_cfg=runtime_motion_cfg)

    glass_gripper_close_mm = _sanitize_gripper_close_mm(gripper_close_map.get("glass", GRIPPER_CLOSE_MM_DEFAULT))
    _append_finish_sequence(
        sequence_api,
        cup_detection=cup_detection,
        cup_gripper_close_mm=glass_gripper_close_mm,
        cup_gripper_open_mm=GRIPPER_OPEN_MM_DEFAULT,
        runtime_cfg=runtime_motion_cfg,
    )

    return {
        "ok": True,
        "status": "planned_sequence",
        "message": "레시피/비전 기반 전체 로봇 시퀀스를 생성했습니다.",
        "plan": {
            "selected_menu": selected_menu,
            "recipe_items": [{"code": c, "amount_ml": float(a)} for c, a in recipe_items],
            "picked_targets": picked_targets,
            "cup_target": cup_detection,
            "runtime_motion_cfg": runtime_motion_cfg,
            "sequence_steps": sequence_api.sequence_steps,
        },
    }


def _load_input(args):
    if args.json_file:
        with open(args.json_file, "r", encoding="utf-8") as fp:
            return json.load(fp)
    raw = sys.stdin.read()
    if not str(raw).strip():
        return {}
    return json.loads(raw)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Bartender robot action planner")
    parser.add_argument("--json-file", default="", help="input JSON file path (optional)")
    args = parser.parse_args(argv)

    try:
        context = _load_input(args)
    except Exception as exc:
        out = {"ok": False, "status": "invalid_input", "message": f"입력 JSON 오류: {exc}"}
        print(json.dumps(out, ensure_ascii=False))
        return 1

    try:
        out = run_robot_action(context)
    except Exception as exc:
        out = {"ok": False, "status": "exception", "message": f"planner 예외: {exc}"}
        print(json.dumps(out, ensure_ascii=False))
        return 1

    print(json.dumps(out, ensure_ascii=False))
    return 0 if bool(out.get("ok", False)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
