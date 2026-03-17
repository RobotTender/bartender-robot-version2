"""Bartender robot action planner.

목적:
- 주문 결과(레시피) + vision1 객체 메타를 바탕으로 로봇 시퀀스를 구성한다.
- 플래너 코드에서는 "데이터 처리"와 "모션 시퀀스"를 분리한다.
- 모션 호출은 DSR 원본 감각과 맞추기 위해 movel(posx(...)), movej(posj(...)) 명시를 따른다.

주의:
- run_robot_action(context) : 기본 진입점 (step 계획 생성)
- run_robot_action(context, api=...) : 외부 API 객체 주입 가능
"""

from __future__ import annotations

import argparse
import json
import math
import sys
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
}


# ---------------------------------------------------------------------------
# 2) 모션 파라미터(여기를 수정하며 튜닝)
# ---------------------------------------------------------------------------

# 사용자 요청에 맞춰 고정값은 아래 시퀀스 함수에서 숫자 리터럴로 직접 표기한다.


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
            "enabled": True,
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
        timeout_sec: float | None = None,
        vel: float | None = None,
        acc: float | None = None,
    ):
        self.movej_posj(joints6=joints6, label=label, timeout_sec=timeout_sec, vel=vel, acc=acc)

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
        # backward compatibility alias: 기존 emergency_stop 호출을 모션 정지로 연결
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
        # backward compatibility helper:
        # 기존 move_to_detection 호출은 "계산 + 이동" 2단계로 풀어서 기록한다.
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


def _pick_detection_for_ingredient(detections, ingredient_code: str):
    if not isinstance(detections, list):
        return None
    aliases = tuple(_norm_code(v) for v in INGREDIENT_CLASS_ALIASES.get(_norm_code(ingredient_code), (ingredient_code,)))
    best = None
    best_score = -1.0
    for det in detections:
        if not isinstance(det, dict):
            continue
        class_name = _norm_code(det.get("class_name"))
        if aliases:
            exact_match = class_name in aliases
            partial_match = any((alias and (alias in class_name or class_name in alias)) for alias in aliases)
            if (not exact_match) and (not partial_match):
                continue

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
            continue

        depth_m = _safe_float(det.get("depth_m"))
        if depth_m is None or depth_m <= 0.0:
            continue
        conf = _safe_float(det.get("confidence"))
        score = float(conf if conf is not None else 0.0)
        if score >= best_score:
            best = {
                "class_name": class_name,
                "confidence": score,
                "center_uv": [float(center_uv[0]), float(center_uv[1])],
                "depth_m": float(depth_m),
            }
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
            "gripper_close_mm": float(grip_map.get(_norm_code(ingredient_code), 41.0)),
            "detection": dict(det),
        }
        resolved_targets.append(row)
        picked_targets.append(dict(row))

    return picked_targets, missing_ingredients, resolved_targets


# ---------------------------------------------------------------------------
# 4) 모션 시퀀스 구성(movel(posx), movej(posj) 의미를 직접 사용)
# ---------------------------------------------------------------------------


def _read_current_posj(api: PlannerSequenceApi):
    """현재 조인트값을 읽어 반환한다.

    반환: (ok: bool, msg: str, posj6: list|None)
    """
    backend = getattr(api, "backend", None)
    if backend is None or (not hasattr(backend, "get_current_positions")):
        return False, "현재 조인트 조회 실패: backend.get_current_positions 미지원", None

    data = backend.get_current_positions()
    if not isinstance(data, (list, tuple)) or len(data) < 2:
        return False, "현재 조인트 조회 실패: 위치 캐시 없음", None

    posj = data[0]
    if not isinstance(posj, (list, tuple)) or len(posj) < 6:
        return False, "현재 조인트 조회 실패: current_posj 길이 부족", None

    try:
        out = [float(v) for v in list(posj)[:6]]
    except Exception as exc:
        return False, f"현재 조인트 조회 실패: 파싱 오류({exc})", None

    return True, "ok", out


def _read_current_posx(api: PlannerSequenceApi):
    """현재 TCP posx값을 읽어 반환한다.

    우선순위:
    1) backend.get_current_posx_live()
    2) backend.get_current_positions()의 current_posx 캐시

    반환: (ok: bool, msg: str, posx6: list|None)
    """
    backend = getattr(api, "backend", None)
    if backend is None:
        return False, "현재 posx 조회 실패: backend 미지원", None

    if hasattr(backend, "get_current_posx_live"):
        posx_live, _sol_live, err_live = backend.get_current_posx_live()
        if err_live is None and isinstance(posx_live, (list, tuple)) and len(posx_live) >= 6:
            try:
                out = [float(v) for v in list(posx_live)[:6]]
            except Exception as exc:
                return False, f"현재 posx 조회 실패: 실시간 파싱 오류({exc})", None
            return True, "ok(live)", out

    if hasattr(backend, "get_current_positions"):
        data = backend.get_current_positions()
        if isinstance(data, (list, tuple)) and len(data) >= 2:
            posx = data[1]
            if isinstance(posx, (list, tuple)) and len(posx) >= 6:
                try:
                    out = [float(v) for v in list(posx)[:6]]
                except Exception as exc:
                    return False, f"현재 posx 조회 실패: 캐시 파싱 오류({exc})", None
                return True, "ok(cache)", out

    return False, "현재 posx 조회 실패: 실시간/캐시 모두 없음", None


def _read_current_pose_to_buffer(api: PlannerSequenceApi, runtime_buf: dict | None = None, key: str = "current_pose"):
    """현재 로봇 pose를 읽어 버퍼에 저장한다.

    저장 형태:
    runtime_buf[key] = {
        "posx": [x, y, z, a, b, c],
        "posj": [j1, j2, j3, j4, j5, j6]
    }

    반환:
    (ok: bool, msg: str, pose: dict|None)
    """
    ok_x, msg_x, cur_posx = _read_current_posx(api)
    if (not ok_x) or (not isinstance(cur_posx, (list, tuple))) or len(cur_posx) < 6:
        return False, str(msg_x), None

    ok_j, msg_j, cur_posj = _read_current_posj(api)
    if (not ok_j) or (not isinstance(cur_posj, (list, tuple))) or len(cur_posj) < 6:
        return False, str(msg_j), None

    pose = {
        "posx": [float(v) for v in list(cur_posx)[:6]],
        "posj": [float(v) for v in list(cur_posj)[:6]],
    }

    if isinstance(runtime_buf, dict):
        runtime_buf[str(key)] = dict(pose)
    return True, "ok", pose


def _run_volume_feedback_pour_control(
    api: PlannerSequenceApi,
    *,
    ingredient_code: str,
    x: float,
    y: float,
    z: float,
    a: float,
    b: float,
    c_start: float,
    target_volume_ml: float,
):
    if not hasattr(api, "backend"):
        raise RuntimeError(f"[{ingredient_code}] 유량제어 실행 실패: backend 실행 경로가 아닙니다.")

    def _recover_to_start_and_stop(reason_text: str):
        recover_err = ""
        try:
            api.movel_posx(
                [float(x), float(y), float(z), float(a), float(b), float(c_start)],
                label=f"[{ingredient_code}] 이상복귀 시작위치",
            )
        except Exception as rec_exc:
            recover_err = f" | 시작복귀 실패: {rec_exc}"
        try:
            api.motion_stop(label=f"[{ingredient_code}] 이상상황 정지")
        except Exception as stop_exc:
            recover_err += f" | 정지 실패: {stop_exc}"
        raise RuntimeError(f"{reason_text}{recover_err}")

    try:
        target_ml = float(target_volume_ml)
        min_c = float(c_start)
        max_c = float(c_start) + 45.0
        step_fast = 0.2
        step_slow = 0.1
        step_down = 0.05
        slow_band_ml = 20.0
        hold_sec = 0.10
        flow_start_delta_ml = 1.0
        flow_start_dv_ml = 0.2
        flow_low_dv_ml = 0.05
        flow_high_dv_ml = 0.45
        no_flow_timeout_sec = 2.5
        max_runtime_sec = 120.0
        min_cmd_delta_deg = 0.03

        baseline_v = float(api.get_current_volume_ml(max_age_sec=0.5))
        last_v = float(baseline_v)
        flow_started = False
        no_flow_started_at = None
        last_cmd_c = float(c_start)
        deadline = time.monotonic() + max_runtime_sec

        while time.monotonic() <= deadline:
            v_now = float(api.get_current_volume_ml(max_age_sec=0.5))
            dv = max(0.0, float(v_now) - float(last_v))
            last_v = float(v_now)
            remain = target_ml - float(v_now)

            if remain <= 0.0:
                api.motion_stop(label=f"[{ingredient_code}] 목표 도달 정지")
                return

            ok_pose_fb, msg_pose_fb, pose_fb = _read_current_pose_to_buffer(api)
            if not ok_pose_fb:
                _recover_to_start_and_stop(f"[{ingredient_code}] 각도 피드백 실패: {msg_pose_fb}")
            c_feedback = float(pose_fb["posx"][5])

            if (not flow_started) and ((float(v_now) - float(baseline_v)) >= flow_start_delta_ml or dv >= flow_start_dv_ml):
                flow_started = True

            target_c = float(c_feedback)
            if flow_started:
                # 출수 시작 이후: 기본 각도 유지, 유량 저하 시에만 미세 증각
                if dv < flow_low_dv_ml:
                    inc = step_slow if remain <= slow_band_ml else step_fast
                    target_c = min(max_c, float(c_feedback) + float(inc))
                elif remain <= slow_band_ml and dv > flow_high_dv_ml:
                    target_c = max(min_c, float(c_feedback) - float(step_down))
            else:
                inc = step_slow if remain <= slow_band_ml else step_fast
                target_c = min(max_c, float(c_feedback) + float(inc))

            # 실패: 최대 기울기인데도 출수 시작이 안됨
            if (not flow_started) and float(c_feedback) >= (float(max_c) - 1e-3) and dv < flow_low_dv_ml:
                if no_flow_started_at is None:
                    no_flow_started_at = time.monotonic()
                elif (time.monotonic() - float(no_flow_started_at)) >= no_flow_timeout_sec:
                    _recover_to_start_and_stop(
                        f"[{ingredient_code}] 취소: 최대기울기({float(c_feedback):.2f})에서도 출수 시작 없음 "
                        f"(dv={float(dv):.2f}ml/{hold_sec:.2f}s)"
                    )
            else:
                no_flow_started_at = None

            if abs(float(target_c) - float(last_cmd_c)) >= min_cmd_delta_deg:
                ok_cmd, msg_cmd = api.backend.send_move_cartesian_async(
                    float(x), float(y), float(z), float(a), float(b), float(target_c), vel=5.0, acc=5.0
                )
                if not ok_cmd:
                    _recover_to_start_and_stop(f"[{ingredient_code}] 유량제어 자세명령 실패: {msg_cmd}")
                last_cmd_c = float(target_c)

            time.sleep(hold_sec)

        _recover_to_start_and_stop(f"[{ingredient_code}] 제어 타임아웃: {max_runtime_sec:.1f}s")
    except RuntimeError:
        raise
    except Exception as exc:
        _recover_to_start_and_stop(f"[{ingredient_code}] 유량제어 예외: {exc}")


def _append_start_sequence(api: PlannerSequenceApi):
    api.set_robot_mode(mode=1, label="로봇 오토모드 전환")
    # 원본 의미: movej(posj(HOME_POSJ), v=..., a=...)
    api.move_home(label="초기 홈 이동")

     # 집기전 자세
    j_target = [28.0, -35.0, 100.0, 77.0, 63.0, -154.0]
    api.movej_posj(j_target, label="[system] 바 쪽 바라보기")
    



def _append_ingredient_sequence(api: PlannerSequenceApi, row: dict, seq_index: int):
  
    ingredient_code = str(row["ingredient_code"])
    center_uv = row["detection"]["center_uv"]
    depth_m = float(row["detection"]["depth_m"])
    target_volume_ml = float(row["target_volume_ml"])
    target_volume_text = f"{target_volume_ml:.1f}ml"
    try:
        gripper_close_mm = float(row.get("gripper_close_mm", 41.0))
        if (not math.isfinite(gripper_close_mm)) or gripper_close_mm < 0.0:
            gripper_close_mm = 41.0
    except Exception:
        gripper_close_mm = 41.0

    # 객체 기준 로봇좌표에서 각 단계별 XYZ 오프셋(mm)을 명시적으로 더해 사용한다.
    pick_base_offset = [0.0, 0.0, 0.0]
    place_top_offset = [0.0, 0.0, 110.0]
    place_offset = [0.0, 0.0, 0.0]
    retreat_offset = [0.0, 0.0, 110.0]
    pour_ready_pose = [430.0, -110.0, 360.0, 180.0, 0.0, 180.0]
    pour_pose_by_ingredient = {
        "soju": [430.0, -95.0, 270.0, 180.0, 0.0, 180.0],
        "beer": [430.0, -130.0, 270.0, 180.0, 0.0, 180.0],
    }
    pour_pose = list(pour_pose_by_ingredient.get(ingredient_code, pour_ready_pose))
    target_key = f"{seq_index}_{ingredient_code}_target"

    api.gripper(109.0, label=f"[{ingredient_code}] 그리퍼 열림")
    api.wait_sec(2.0, label="그리퍼 열림 대기")

    # 바 바라보기
    # j_target = [28.0, -35.0, 100.0, 77.0, 63.0, -154.0]
    # api.movej_posj(j_target, label=f"[{ingredient_code}] 바 쪽 바라보기")
    
    
    # A) 비전1 대상 병 접근 -> 파지 -> 리프트
    # 역할: 비전 검출값(center_uv, depth_m)을 로봇 좌표계 목표로 변환해 `target_key`로 1회 저장한다.
    api.resolve_detection_target(
        target_key=target_key,
        ingredient_code=ingredient_code,
        center_uv=center_uv,
        depth_m=depth_m,
        label=f"[{ingredient_code}] 병 기준 타겟 계산",
        extra_offset_xyz_mm=pick_base_offset,
        apply_menu_offset=True,
    )
    # 비전 목표위치 획득.
    base_xyz = getattr(api, "_resolved_targets", {}).get(target_key)
    if base_xyz is None:
        raise RuntimeError(f"target_key={target_key} 미해결")
    x, y, z = [float(v) for v in base_xyz[:3]]

    # 현재위치 (posx/posj)를 버퍼로 읽는다.    # 필요할 때마다 같은 패턴으로 재호출하면 된다.
    ok_pose, msg_pose, pose_now = _read_current_pose_to_buffer(api)
    if not ok_pose:
        # posx/posj는 모두 필수 값이다. 하나라도 없으면 시퀀스를 중단한다.
        raise RuntimeError(f"[{ingredient_code}] 시퀀스 중지: {msg_pose}")
    cur_posx = list(pose_now["posx"])
    cur_posj = list(pose_now["posj"])

    a = float(cur_posx[3])
    b = float(cur_posx[4])
    c = float(cur_posx[5])

    # 픽 접근 위치정의(절대좌표 XYZABC)
    pick_approach_pose = [
        x,
        y-100.0,
        z,
        a,
        b,
        c,
    ]
    # 비전 목표위치 이동
    api.movel_posx(pick_approach_pose, label=f"[{ingredient_code}] 병 접근(절대좌표)")

    # 병 파지 위치(절대좌표 XYZABC)
    pick_pose = [
        x,
        y,
        z,
        a,
        b,
        c,
    ]
    api.movel_posx(pick_pose, label=f"[{ingredient_code}] 병 파지 위치 이동")
    # 역할: 그리퍼를 닫아 병을 파지한다.
    api.gripper(gripper_close_mm, label=f"[{ingredient_code}] 병 파지")
    api.wait_sec(2.0, label="그리퍼 파지 대기")

    # 역할: 기준 타겟 + 리프트 오프셋(절대좌표 XYZABC)으로 이동한다.
    pick_lift_pose = [
        x ,
        y ,
        z + 30.0,
        a,
        b,
        c,
    ]
    # 역할: 병을 든 상태로 리프트 포즈까지 이동한다.
    api.movel_posx(pick_lift_pose, label=f"[{ingredient_code}] 병 리프트 위치 이동")
    

    # B) 따르기 위치 이동 -> 비전2 목표용량까지 대기
    # 원본 의미: movel(posx([430,-110,360,180,0,180]), vel=velx, acc=accx, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    # 역할: 공통 따르기 준비 포즈로 이동한다.

    # 역할: 책상에 부딪히지 않게 책상밖으로 나온다.
    pick_return_pose1 = [
        x ,
        y - 300.0,
        z + 30.0,
        a,
        b,
        c,
    ]
    api.movel_posx(pick_return_pose1, label=f"[{ingredient_code}] 병 붓는 대기위치1이동")

    pick_return_pose2 = [
        x ,
        y - 300.0,
        z - 100.0,
        a,
        b,
        c,
    ]
    api.movel_posx(pick_return_pose2, label=f"[{ingredient_code}] 병 붓는 대기위치2이동")

    # 따르기 전 자세
    pour_wait_pos1 = [45.0, 0.0, 135.0, 90.0, -90.0, -135.0]
    api.movej_posj(pour_wait_pos1, label=f"[{ingredient_code}] 따르기준비 위치 이동")
    
     # 따르기 자세
    pour_wait_pos2 = [45.0, 0.0, 135.0, 90.0, -90.0, -135.0]
    api.movej_posj(pour_wait_pos2, label=f"[{ingredient_code}] 따르기 위치 이동")


     # 현재위치 (posx/posj)를 버퍼로 읽는다.    # 필요할 때마다 같은 패턴으로 재호출하면 된다.
    ok_pose, msg_pose, pose_now = _read_current_pose_to_buffer(api)
    if not ok_pose:
        # posx/posj는 모두 필수 값이다. 하나라도 없으면 시퀀스를 중단한다.
        raise RuntimeError(f"[{ingredient_code}] 시퀀스 중지: {msg_pose}")
    cur_posx = list(pose_now["posx"])
    cur_posj = list(pose_now["posj"])

    x = float(cur_posx[0])
    y = float(cur_posx[1])
    z = float(cur_posx[2])
    a = float(cur_posx[3])
    b = float(cur_posx[4])
    c = float(cur_posx[5])
    pour_pose = [x,y,z,a,b,c]

    api.movel_posx(pour_pose, label=f"[{ingredient_code}] 병 붓는 위치 이동")

    _run_volume_feedback_pour_control(
        api,
        ingredient_code=ingredient_code,
        x=x,
        y=y,
        z=z,
        a=a,
        b=b,
        c_start=c,
        target_volume_ml=target_volume_ml,
    )

    # ----------------------------------------------------------------------
    # 예제(주석): 용량 피드백 기반 amovel 미세 따르기 루프
    # - 다른 로직 영향 방지를 위해 실제 실행은 하지 않음.
    # - 아이디어: 목표치에 가까워질수록 c(기울기) 증가폭을 줄여 오버슈트 최소화.
    
    # target_ml = float(target_volume_ml)
    # min_c = float(c)             # 시작 기울기
    # max_c = float(c) + 45.0      # 최대 기울기
    # cur_c = min_c
    # step_fast = 0.2              # 목표와 멀 때(ml) 각도 증가량
    # step_slow = 0.1              # 목표 근처 각도 증가량
    # slow_band_ml = 20.0          # 목표 근처 감속 구간
    # return_margin_ml = 5.0       # 목표 직전 복귀 시작 여유
    # no_flow_eps_ml = 0.8         # 유량 판정 최소 증가량(ml)
    # no_flow_timeout_sec = 2.5    # 최대기울기에서 무유량 허용 시간
    # hold_sec = 0.10              # 샘플 주기(현재 프로그램 기준 권장값)
    #
    # last_v = api.get_current_volume_ml(max_age_sec=0.5)
    # no_flow_started_at = None
    #
    # while True:
    #     v_now = api.get_current_volume_ml(max_age_sec=0.5)
    #     remain = target_ml - float(v_now)
    #
    #     # 1) 목표 도달
    #     if remain <= 0.0:
    #         api.motion_stop(label=f"[{ingredient_code}] 목표 도달 정지")
    #         break
    #
    #     # 2) 목표 근접 시 선복귀(오버슈트 방지)
    #     if remain <= return_margin_ml:
    #         api.amovel_posx(
    #             [x, y, z + 10.0, a, b, min_c],
    #             label=f"[{ingredient_code}] 미세 복귀",
    #             vel=10.0,
    #             acc=10.0,
    #         )
    #         api.motion_stop(label=f"[{ingredient_code}] 목표 근접 정지")
    #         break
    #
    #     # 3) 목표 근처에서는 작은 스텝으로 천천히 기울임
    #     dtheta = step_slow if remain <= slow_band_ml else step_fast
    #     cur_c = min(max_c, cur_c + dtheta)
    #
    #     # 4) c만 갱신해서 비동기 자세 명령(중첩 발행)
    #     #    목표 근처에서는 속도도 줄여 미세 제어
    #     if remain <= slow_band_ml:
    #         api.amovel_posx([x, y, z, a, b, cur_c], label=f"[{ingredient_code}] 미세 따르기 c={cur_c:.2f}", vel=12.0, acc=12.0)
    #     else:
    #         api.amovel_posx([x, y, z, a, b, cur_c], label=f"[{ingredient_code}] 미세 따르기 c={cur_c:.2f}")
    #
    #     # 5) 최대 기울기인데 유량 증가가 없으면 취소(재료 없음/출수 불량)
    #     dv = float(v_now) - float(last_v)
    #     last_v = float(v_now)
    #     if cur_c >= (max_c - 1e-6):
    #         if dv < no_flow_eps_ml:
    #             if no_flow_started_at is None:
    #                 no_flow_started_at = time.monotonic()
    #             elif (time.monotonic() - no_flow_started_at) >= no_flow_timeout_sec:
    #                 api.motion_stop(label=f"[{ingredient_code}] 무유량 취소 정지")
    #                 raise RuntimeError(
    #                     f"[{ingredient_code}] 취소: 최대기울기({cur_c:.2f})에서도 유량 증가 없음 "
    #                     f"(dv={dv:.2f}ml, timeout={no_flow_timeout_sec:.1f}s)"
    #                 )
    #         else:
    #             no_flow_started_at = None
    #
    #     api.wait_sec(hold_sec, label=f"[{ingredient_code}] 샘플 대기")
    # ----------------------------------------------------------------------
  

    '''
    api.movel_posx(pour_pose, label=f"[{ingredient_code}] 병 붓는 위치 이동")

    # 역할: 따르기 완료 후 안전한 준비 포즈로 복귀한다.
    # 따르기 전 자세
    
    api.movej_posj(pour_wait_pos1, label=f"[{ingredient_code}] 따르기준비 위치 이동")

    api.movel_posx(pick_return_pose2, label=f"[{ingredient_code}] 병 붓는 대기위치2이동")

    api.movel_posx(pick_return_pose1, label=f"[{ingredient_code}] 병 붓는 대기위치1이동")

    api.movel_posx(pick_lift_pose, label=f"[{ingredient_code}] 병 리프트 위치 이동")

    api.movel_posx(pick_pose, label=f"[{ingredient_code}] 병 파지 위치 이동", vel=10.0, acc=10.0)

    api.gripper(gripper_open_mm, label=f"[{ingredient_code}] 그리퍼 오픈")

    api.wait_sec(2.0, label=f"[{ingredient_code}] 그리퍼 오픈 대기")

    api.movel_posx(pick_approach_pose, label=f"[{ingredient_code}] 병 접근(절대좌표)")

    '''

def _append_finish_sequence(
    api: PlannerSequenceApi,
    ingredient_code: str = "glass",
    gripper_close_mm: float = 41.0,
    gripper_open_mm: float = 90.0,
):
    # 바 앞에서 복귀하기
    # api.gripper(gripper_open_mm, label=f"[{ingredient_code}] 그리퍼 오픈")
    # api.wait_sec(2.0, label=f"[{ingredient_code}] 그리퍼 오픈 대기")
    # j_target = [28.0, -35.0, 100.0, 77.0, 63.0, -154.0]
    # api.movej_posj(j_target, label=f"[{ingredient_code}] 바 쪽 바라보기")

    # 완성컵 집기 위치 이동(고정좌표 or 비전좌표)
    # j_target = [28.0, -35.0, 100.0, 77.0, 63.0, -154.0]
    # api.movej_posj(j_target, label=f"[{ingredient_code}] 완성잔 집기 대기위치 이동")
    # api.movel_posx([520.0, -20.0, 300.0, 180.0, 0.0, 180.0], label=f"[{ingredient_code}] 완성잔 집기위치 이동")
    # api.gripper(gripper_close_mm, label=f"[{ingredient_code}] 그리퍼 파지")
    # api.wait_sec(2.0, label=f"[{ingredient_code}] 그리퍼 파지 대기")

    # 전달 위치 이동(사람 앞)
    # j_target = [28.0, -35.0, 100.0, 77.0, 63.0, -154.0]
    # api.movej_posj(j_target, label=f"[{ingredient_code}] 완성잔 전달 대기위치1 이동")
    # j_target = [28.0, -35.0, 100.0, 77.0, 63.0, -154.0]
    # api.movej_posj(j_target, label=f"[{ingredient_code}] 완성잔 전달 대기위치2 이동")
    # api.movel_posx([520.0, -20.0, 300.0, 180.0, 0.0, 180.0], label=f"[{ingredient_code}] 완성잔 전달위치 업 이동")
    # api.movel_posx([520.0, -20.0, 300.0, 180.0, 0.0, 180.0], label=f"[{ingredient_code}] 완성잔 전달위치 이동")
    # api.gripper(gripper_open_mm, label=f"[{ingredient_code}] 그리퍼 오픈")
    # api.wait_sec(2.0, label=f"[{ingredient_code}] 그리퍼 오픈 대기")
    # api.movel_posx([520.0, -20.0, 300.0, 180.0, 0.0, 180.0], label=f"[{ingredient_code}] 완성잔 전달위치 업 이동")

    # 복귀
    # j_target = [28.0, -35.0, 100.0, 77.0, 63.0, -154.0]
    # api.movej_posj(j_target, label=f"[{ingredient_code}] 완성잔 전달 대기위치2 이동")
    # j_target = [28.0, -35.0, 100.0, 77.0, 63.0, -154.0]
    # api.movej_posj(j_target, label=f"[{ingredient_code}] 완성잔 전달 대기위치1 이동")

    # 홈위치
    # api.move_home(label=f"[{ingredient_code}] 시퀀스 종료 홈 복귀")

    pass


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

    picked_targets, missing_ingredients, resolved_targets = _resolve_targets_from_recipe(
        recipe_items,
        detections,
        gripper_close_map=gripper_close_map,
    )
    if missing_ingredients:
        return {
            "ok": False,
            "status": "vision_target_missing",
            "message": f"vision1에서 레시피 재료를 찾지 못했습니다: {', '.join(missing_ingredients)}",
            "missing_ingredients": list(missing_ingredients),
            "plan": {
                "selected_menu": selected_menu,
                "recipe_items": [{"code": c, "amount_ml": float(a)} for c, a in recipe_items],
                "picked_targets": picked_targets,
                "sequence_steps": sequence_api.sequence_steps,
            },
        }

    _append_start_sequence(sequence_api)
    for idx, row in enumerate(resolved_targets, start=1):
        _append_ingredient_sequence(sequence_api, row, seq_index=idx)
    glass_gripper_close_mm = float(gripper_close_map.get("glass", 41.0))
    _append_finish_sequence(
        sequence_api,
        ingredient_code="glass",
        gripper_close_mm=glass_gripper_close_mm,
        gripper_open_mm=90.0,
    )

    return {
        "ok": True,
        "status": "planned_sequence",
        "message": "레시피/비전 기반 전체 로봇 시퀀스를 생성했습니다.",
        "plan": {
            "selected_menu": selected_menu,
            "recipe_items": [{"code": c, "amount_ml": float(a)} for c, a in recipe_items],
            "picked_targets": picked_targets,
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
