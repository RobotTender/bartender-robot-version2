import base64
import copy
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse


SEQUENCE_STEP_KEYS = (
    "boot",
    "mode",
    "voice_request",
    "stt",
    "llm",
    "recipe",
    "robot_action",
    "vision_check",
    "done",
)
SEQUENCE_STEP_LABELS = {
    "boot": "시스템 준비",
    "mode": "모드 확인",
    "voice_request": "음성 입력 요청",
    "stt": "STT 인식",
    "llm": "LLM 주문 판별",
    "recipe": "레시피 생성",
    "robot_action": "로봇 동작",
    "vision_check": "비전 확인",
    "done": "완료",
}

ROBOT_ERROR_STATE_CODES = {3, 5, 6, 7, 10, 11, 15}
VISION_META_STALE_SEC_DEFAULT = 2.0
LOG_KEEP_MAX = 400
# retry(추천 후 재질문)만 자동 재입력을 허용한다.
VOICE_ORDER_RETRY_MAX_ATTEMPTS = max(
    1, int(float(os.environ.get("BARTENDER_VOICE_RETRY_MAX_ATTEMPTS", "3")))
)
VOICE_ORDER_RUNTIME_TIMEOUT_SEC = max(
    10.0, float(os.environ.get("BARTENDER_VOICE_RUNTIME_TIMEOUT_SEC", "90.0"))
)
PRECHECK_MODE_SYNC_INTERVAL_SEC = max(
    0.2, float(os.environ.get("BARTENDER_PRECHECK_MODE_SYNC_INTERVAL_SEC", "1.2"))
)
PRECHECK_MODE_SYNC_TIMEOUT_SEC = max(
    0.05, float(os.environ.get("BARTENDER_PRECHECK_MODE_SYNC_TIMEOUT_SEC", "0.25"))
)

RECIPE_INGREDIENT_ALIASES = {
    "soju": {"soju", "소주"},
    "beer": {"beer", "맥주"},
}


def _now_iso_text() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


def _normalize_mode(mode: str) -> str:
    value = str(mode or "").strip().lower()
    return "auto" if value == "auto" else "manual"


class BartenderSequenceManager:
    def __init__(self, backend):
        self._backend = backend
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._run_id = 0
        self._last_log_id = 0
        self._vision_meta_stale_sec = max(
            0.2,
            float(
                getattr(
                    backend,
                    "VISION1_META_STALE_SEC",
                    VISION_META_STALE_SEC_DEFAULT,
                )
                if hasattr(backend, "VISION1_META_STALE_SEC")
                else VISION_META_STALE_SEC_DEFAULT
            ),
        )
        self._state = self._empty_state()
        self._tts_done_run_id = 0
        self._tts_done_at = 0.0
        self._last_precheck_mode_sync_at = 0.0
        self._stop_motion_lock = threading.Lock()
        self._stop_motion_req_id = 0

    def _empty_state(self) -> dict:
        return {
            "run_id": 0,
            "running": False,
            "mode": "manual",
            "status": "idle",
            "message": "대기",
            "active_step": "",
            "step_states": {k: "pending" for k in SEQUENCE_STEP_KEYS},
            "logs": [],
            "started_at": "",
            "finished_at": "",
            "updated_at": _now_iso_text(),
            "request": {},
            "last_order_result": {},
            "last_error_step": "",
            "last_error_step_label": "",
            "last_error_stage": "",
            "last_error_message": "",
            "stop_requested": False,
            "safety_reason": "",
            "voice_mic_level": 0,
        }

    def _append_log_locked(self, stage: str, message: str, level: str = "info", data: Any = None):
        text = str(message or "").strip()
        if not text:
            return
        self._last_log_id += 1
        row = {
            "id": int(self._last_log_id),
            "time": _now_iso_text(),
            "level": str(level or "info").strip().lower(),
            "stage": str(stage or "system").strip().lower(),
            "message": text,
        }
        if data is not None:
            row["data"] = data
        logs = list(self._state.get("logs", []) or [])
        logs.append(row)
        if len(logs) > LOG_KEEP_MAX:
            logs = logs[-LOG_KEEP_MAX:]
        self._state["logs"] = logs
        self._state["updated_at"] = _now_iso_text()

    def _set_step_state_locked(self, step_key: str, state: str):
        key = str(step_key or "").strip()
        if key not in self._state["step_states"]:
            return
        self._state["step_states"][key] = str(state or "pending").strip().lower()
        self._state["updated_at"] = _now_iso_text()

    def _set_active_step_locked(self, step_key: str):
        key = str(step_key or "").strip()
        if key not in self._state["step_states"]:
            return
        prev = str(self._state.get("active_step", "") or "").strip()
        if prev and prev in self._state["step_states"]:
            if self._state["step_states"].get(prev) == "active":
                self._state["step_states"][prev] = "done"
        self._state["active_step"] = key
        self._state["step_states"][key] = "active"
        if bool(self._state.get("running", False)):
            step_label = str(SEQUENCE_STEP_LABELS.get(key, key) or key)
            self._state["message"] = f"진행중: {step_label}"
        self._state["updated_at"] = _now_iso_text()

    def _finalize_locked(self, success: bool, message: str, *, stopped: bool = False, safety_reason: str = ""):
        active = str(self._state.get("active_step", "") or "").strip()
        if active:
            self._set_step_state_locked(active, "done" if success else "error")
            self._state["active_step"] = ""
        self._state["running"] = False
        self._state["status"] = "stopped" if stopped else ("success" if success else "error")
        self._state["message"] = str(message or "").strip() or ("완료" if success else "실패")
        self._state["voice_mic_level"] = 0
        self._state["finished_at"] = _now_iso_text()
        self._state["updated_at"] = _now_iso_text()
        if success:
            self._state["last_error_step"] = ""
            self._state["last_error_step_label"] = ""
            self._state["last_error_stage"] = ""
            self._state["last_error_message"] = ""
        if safety_reason:
            self._state["safety_reason"] = str(safety_reason)

    def get_snapshot(self) -> dict:
        with self._lock:
            return copy.deepcopy(self._state)

    def start(self, mode: str = "manual", request: dict | None = None):
        req = dict(request or {})
        norm_mode = _normalize_mode(mode)
        with self._lock:
            if bool(self._state.get("running", False)):
                return False, "시퀀스가 이미 실행 중입니다.", copy.deepcopy(self._state)
            self._run_id += 1
            run_id = int(self._run_id)
            self._state = self._empty_state()
            self._state["run_id"] = run_id
            self._state["running"] = True
            self._state["mode"] = norm_mode
            self._state["status"] = "running"
            self._state["message"] = "시퀀스 실행중"
            self._state["started_at"] = _now_iso_text()
            self._state["request"] = req
            self._state["stop_requested"] = False
            self._tts_done_run_id = 0
            self._tts_done_at = 0.0
            self._append_log_locked("system", f"시퀀스 시작(mode={norm_mode})")
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run_sequence_worker,
                args=(run_id, norm_mode, req),
                daemon=True,
            )
            self._thread.start()
            return True, "시퀀스 시작", copy.deepcopy(self._state)

    def stop(self, reason: str = ""):
        text = str(reason or "").strip() or "사용자 중지 요청"
        with self._lock:
            if not bool(self._state.get("running", False)):
                return False, "실행 중인 시퀀스가 없습니다.", copy.deepcopy(self._state)
            self._state["stop_requested"] = True
            self._state["status"] = "stopping"
            self._state["message"] = text
            self._append_log_locked("system", text, level="warning")
        self._stop_event.set()
        self._request_robot_motion_stop_async(trigger="sequence_stop")
        return True, text, self.get_snapshot()

    def reset(self, reason: str = ""):
        text = str(reason or "").strip() or "시퀀스 상태 초기화"
        thread = None
        with self._lock:
            if bool(self._state.get("running", False)):
                self._state["stop_requested"] = True
                self._state["status"] = "stopping"
                self._state["message"] = text
                self._append_log_locked("system", f"{text}: 실행중 시퀀스 중지", level="warning")
                thread = self._thread
                self._stop_event.set()
                self._request_robot_motion_stop_async(trigger="sequence_reset")
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.2)
        with self._lock:
            self._state = self._empty_state()
            self._state["message"] = text
            self._state["updated_at"] = _now_iso_text()
            self._thread = None
            self._tts_done_run_id = 0
            self._tts_done_at = 0.0
            self._stop_event.clear()
            return True, text, copy.deepcopy(self._state)

    def notify_tts_done(self, run_id: int | None = None):
        with self._lock:
            current_run_id = int(self._state.get("run_id", 0) or 0)
            target_run_id = int(current_run_id if run_id is None else run_id)
            if target_run_id <= 0:
                return False, "유효하지 않은 run_id", copy.deepcopy(self._state)
            self._tts_done_run_id = int(target_run_id)
            self._tts_done_at = time.monotonic()
            if bool(self._state.get("running", False)) and int(current_run_id) == int(target_run_id):
                self._append_log_locked("tts", f"TTS 완료 플래그 수신(run_id={target_run_id})")
            return True, "TTS 완료 플래그 반영", copy.deepcopy(self._state)

    def shutdown(self):
        self._stop_event.set()
        self._request_robot_motion_stop_async(trigger="sequence_shutdown")
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.5)

    def _request_robot_motion_stop_async(self, trigger: str = "sequence_stop"):
        backend = self._backend
        if backend is None or (not hasattr(backend, "send_motion_stop")):
            return
        try:
            with self._stop_motion_lock:
                self._stop_motion_req_id += 1
                req_id = int(self._stop_motion_req_id)
        except Exception:
            req_id = 0

        def _job():
            ok = False
            msg = ""
            try:
                ok, msg = backend.send_motion_stop(stop_mode=2)
            except Exception as exc:
                ok, msg = False, f"예외: {exc}"
            level = "warning" if ok else "error"
            text = (
                f"중지요청({str(trigger or 'sequence_stop')}) 모션정지 "
                f"{'완료' if ok else '실패'}: {str(msg or '-')}"
            )
            with self._lock:
                if req_id and req_id < int(self._stop_motion_req_id):
                    return
                self._append_log_locked("system", text, level=level)

        threading.Thread(target=_job, daemon=True).start()

    def _check_safety_reason(self, mode: str, require_vision: bool = False):
        if self._stop_event.is_set():
            return "중지 요청 수신"
        backend = self._backend
        if backend is None:
            return "백엔드 인스턴스가 없습니다."
        try:
            if hasattr(backend, "is_ready") and (not bool(backend.is_ready())):
                return "백엔드 준비 중입니다."
        except Exception:
            return "백엔드 준비상태 확인 실패"

        if hasattr(backend, "get_robot_state_snapshot"):
            try:
                state_code, state_name, _seen = backend.get_robot_state_snapshot()
                code = None if state_code is None else int(state_code)
            except Exception:
                code, state_name = None, ""
            if code is not None and code in ROBOT_ERROR_STATE_CODES:
                return f"로봇 에러 상태: {state_name or f'STATE_{code}'}({code})"

        if require_vision:
            if not hasattr(backend, "get_vision1_meta_snapshot"):
                return "비전 메타 조회 함수를 찾지 못했습니다."
            meta_payload, seen_at = backend.get_vision1_meta_snapshot()
            if meta_payload is None or seen_at is None:
                return "비전1 메타데이터 미수신"
            age = time.monotonic() - float(seen_at)
            if age > float(self._vision_meta_stale_sec):
                return f"비전1 메타데이터 지연: {age * 1000.0:.0f}ms"
        return None

    def _check_mode_reason(self, mode: str):
        backend = self._backend
        if backend is None:
            return "백엔드 인스턴스가 없습니다."
        target_mode = 1 if _normalize_mode(mode) == "auto" else 0

        now = time.monotonic()
        if (now - float(self._last_precheck_mode_sync_at)) >= float(PRECHECK_MODE_SYNC_INTERVAL_SEC):
            self._last_precheck_mode_sync_at = now
            try:
                if hasattr(backend, "sync_robot_mode_once"):
                    backend.sync_robot_mode_once(force=False, timeout_sec=PRECHECK_MODE_SYNC_TIMEOUT_SEC)
            except Exception:
                pass

        if not hasattr(backend, "get_robot_mode_snapshot"):
            return "로봇모드 조회 함수를 찾지 못했습니다."
        try:
            mode_value, mode_seen_at = backend.get_robot_mode_snapshot()
        except Exception:
            return "로봇모드 확인 실패"
        if mode_value is None or mode_seen_at is None:
            return "로봇모드 미수신"
        try:
            mode_int = int(mode_value)
        except Exception:
            return "로봇모드 값이 올바르지 않습니다."
        if mode_int != int(target_mode):
            return f"현재 로봇모드={mode_int} (목표={int(target_mode)})"
        return None

    def get_precheck(self, mode: str = "auto"):
        norm_mode = _normalize_mode(mode)
        # manual/auto 동일 조건: 실행 전 점검은 공통 안전조건만 사용한다.
        system_reason = self._check_safety_reason(mode=norm_mode, require_vision=False)
        mode_reason = None
        system_ready = system_reason is None
        mode_ready = mode_reason is None
        return {
            "ok": True,
            "mode": norm_mode,
            "ready": bool(system_ready and mode_ready),
            "system_ready": bool(system_ready),
            "system_reason": str(system_reason or ""),
            "mode_ready": bool(mode_ready),
            "mode_reason": str(mode_reason or ""),
            "checked_at": _now_iso_text(),
        }

    def _missing_ingredients(self, recipe: dict, vision_meta_payload: dict):
        if not isinstance(recipe, dict) or not recipe:
            return []
        detections = []
        if isinstance(vision_meta_payload, dict):
            detections = vision_meta_payload.get("detections", [])
        present = set()
        if isinstance(detections, list):
            for det in detections:
                if not isinstance(det, dict):
                    continue
                name = str(det.get("class_name", "") or "").strip().lower()
                if name:
                    present.add(name)
        missing = []
        for key, value in recipe.items():
            code = str(key or "").strip().lower()
            if not code:
                continue
            try:
                amount = float(value)
            except Exception:
                amount = 0.0
            if amount <= 0.0:
                continue
            aliases = set(RECIPE_INGREDIENT_ALIASES.get(code, set()))
            aliases.add(code)
            if not any(alias in present for alias in aliases):
                missing.append(code)
        return missing

    def _sync_step_from_stage(self, stage: str):
        key = str(stage or "").strip().lower()
        if key in ("input_request",):
            mapped = "voice_request"
        elif key in ("stt_wait", "stt_process", "stt"):
            mapped = "stt"
        elif key in ("classify", "llm"):
            mapped = "llm"
        elif key in ("recipe",):
            mapped = "recipe"
        else:
            mapped = ""
        if not mapped:
            return
        self._set_active_step(mapped)

    def _set_active_step(self, key: str):
        with self._lock:
            self._set_active_step_locked(key)

    def _set_step_done_if_pending(self, key: str):
        with self._lock:
            current = str(self._state["step_states"].get(key, "pending") or "pending").strip().lower()
            if current in ("pending", "active"):
                self._set_step_state_locked(key, "done")

    def _append_log(self, stage: str, message: str, level: str = "info", data: Any = None):
        with self._lock:
            self._append_log_locked(stage, message, level=level, data=data)

    def _fail(self, run_id: int, message: str, safety_reason: str = "", stage: str = ""):
        with self._lock:
            if int(self._state.get("run_id", -1)) != int(run_id):
                return
            active = str(self._state.get("active_step", "") or "").strip()
            stage_text = str(stage or "").strip().lower()
            log_stage = stage_text or active or "error"
            self._state["last_error_step"] = active
            self._state["last_error_step_label"] = str(SEQUENCE_STEP_LABELS.get(active, active) or active)
            self._state["last_error_stage"] = log_stage
            self._state["last_error_message"] = str(message or "").strip()
            self._append_log_locked(log_stage, message, level="error")
            self._finalize_locked(False, message, stopped=False, safety_reason=safety_reason)

    def _succeed(self, run_id: int, message: str):
        with self._lock:
            if int(self._state.get("run_id", -1)) != int(run_id):
                return
            self._set_step_state_locked("done", "done")
            self._append_log_locked("done", message, level="info")
            self._finalize_locked(True, message)

    def _stop_finalize(self, run_id: int, message: str):
        with self._lock:
            if int(self._state.get("run_id", -1)) != int(run_id):
                return
            self._append_log_locked("system", message, level="warning")
            self._finalize_locked(False, message, stopped=True, safety_reason=message)

    def _guard_or_stop(self, run_id: int, mode: str, require_vision: bool = False):
        reason = self._check_safety_reason(mode=mode, require_vision=require_vision)
        if reason is None:
            return True
        if "중지 요청" in str(reason):
            self._stop_finalize(run_id, str(reason))
            return False
        self._fail(run_id, str(reason), safety_reason=str(reason))
        return False

    def _run_sequence_worker(self, run_id: int, mode: str, request: dict):
        backend = self._backend
        try:
            voice_only = bool(request.get("voice_only", False))
            robot_action_only = bool(request.get("robot_action_only", False))
            mode_norm = _normalize_mode(mode)
            self._set_active_step("boot")
            if not self._guard_or_stop(run_id, mode):
                return
            self._set_step_done_if_pending("boot")

            self._set_active_step("mode")
            if voice_only:
                self._append_log("mode", "voice_only 실행: 로봇모드 전환 스킵")
            elif robot_action_only:
                self._append_log("mode", "robot_action_only 실행: 로봇모드 전환 스킵")
            else:
                # 메뉴얼/오토 동일 동작: 모드 전환 호출은 하지 않고 공통으로 스킵한다.
                self._append_log("mode", "모드 전환 스킵(메뉴얼/오토 동일)")
            self._set_step_done_if_pending("mode")
            if not self._guard_or_stop(run_id, mode):
                return

            if robot_action_only:
                manual_mode = (mode_norm == "manual")
                if not manual_mode:
                    self._fail(run_id, "로봇동작 단독 테스트는 메뉴얼모드에서만 실행할 수 있습니다.")
                    return
                order_result = request.get("order_result", {})
                if not isinstance(order_result, dict):
                    order_result = {}
                order_result = dict(order_result)
                if str(order_result.get("status", "") or "").strip().lower() != "success":
                    self._fail(run_id, "로봇동작 단독 테스트에 사용할 주문 결과가 없습니다.")
                    return
                if not self._guard_or_stop(run_id, mode, require_vision=True):
                    return

                self._set_step_done_if_pending("voice_request")
                self._set_step_done_if_pending("stt")
                self._set_step_done_if_pending("llm")
                self._set_step_done_if_pending("recipe")

                self._set_active_step("robot_action")
                menu_offsets = request.get("menu_offsets")
                motion_speed_percent = request.get("motion_speed_percent")
                ok_action, action_msg = backend.run_bartender_first_ingredient_action(
                    order_result,
                    menu_offsets=menu_offsets,
                    motion_speed_percent=motion_speed_percent,
                    execute_enabled_override=bool(request.get("execute_robot_action", True)),
                )
                if not ok_action:
                    self._fail(run_id, f"로봇동작 실패: {action_msg}")
                    return
                self._append_log("robot_action", str(action_msg or "로봇동작 완료"))
                self._set_step_done_if_pending("robot_action")

                self._set_active_step("vision_check")
                self._set_step_done_if_pending("vision_check")
                self._set_active_step("done")
                self._succeed(run_id, "로봇동작 단독 테스트 완료")
                return

            self._set_active_step("voice_request")
            input_text = str(request.get("input_text", "") or "").strip()
            recommend_menu = str(request.get("recommend_menu", "") or "").strip()
            allow_llm = bool(request.get("allow_llm", True))
            request_stt = bool(request.get("request_stt", (not bool(input_text))))
            audio_filename = str(request.get("audio_filename", "recording.webm") or "recording.webm").strip() or "recording.webm"
            audio_bytes = None
            raw_audio_b64 = str(request.get("audio_base64", "") or "").strip()
            if raw_audio_b64:
                try:
                    audio_bytes = base64.b64decode(raw_audio_b64, validate=True)
                except Exception as exc:
                    self._fail(run_id, f"오디오 payload 디코딩 실패: {exc}")
                    return
            max_attempts = int(request.get("voice_retry_max_attempts", VOICE_ORDER_RETRY_MAX_ATTEMPTS))
            if max_attempts <= 0:
                max_attempts = int(VOICE_ORDER_RETRY_MAX_ATTEMPTS)

            current_input_text = str(input_text)
            current_recommend_menu = str(recommend_menu)
            current_request_stt = bool(request_stt)
            current_audio_bytes = audio_bytes
            current_audio_filename = str(audio_filename)
            status = ""
            result = {}
            ok_voice = False
            voice_msg = ""

            for attempt in range(1, int(max_attempts) + 1):
                if not self._guard_or_stop(run_id, mode):
                    return
                self._append_log("voice_request", f"음성주문 요청 시도 {attempt}/{max_attempts}")
                streamed_non_mic_events = 0

                def _on_voice_event(evt):
                    nonlocal streamed_non_mic_events
                    if not isinstance(evt, dict):
                        return
                    evt_type = str(evt.get("type", "") or "").strip().lower()
                    if evt_type == "mic_level":
                        try:
                            mic_level = int(evt.get("level", 0) or 0)
                        except Exception:
                            mic_level = 0
                        mic_level = max(0, min(100, int(mic_level)))
                        with self._lock:
                            if int(self._state.get("run_id", -1)) == int(run_id):
                                self._state["voice_mic_level"] = int(mic_level)
                                self._state["updated_at"] = _now_iso_text()
                        return
                    streamed_non_mic_events += 1
                    stage = str(evt.get("stage", evt_type) or evt_type).strip().lower()
                    message = str(evt.get("message", "") or "").strip()
                    data = evt.get("data")
                    level = str(evt.get("severity", "info") or "info").strip().lower()
                    if data is not None:
                        try:
                            message = f"{message} | {json.dumps(data, ensure_ascii=False)}"
                        except Exception:
                            message = f"{message} | {data}"
                    self._append_log(stage or "voice", message or "-", level=level)
                    if evt_type == "stage":
                        self._sync_step_from_stage(stage)

                ok_voice, voice_payload, voice_msg = backend.run_voice_order_runtime(
                    input_text=current_input_text,
                    recommend_menu=current_recommend_menu,
                    allow_llm=allow_llm,
                    request_stt=current_request_stt,
                    audio_bytes=current_audio_bytes,
                    audio_filename=current_audio_filename,
                    timeout_sec=float(VOICE_ORDER_RUNTIME_TIMEOUT_SEC),
                    cancel_checker=lambda: bool(self._stop_event.is_set()),
                    event_callback=_on_voice_event,
                )
                if self._stop_event.is_set():
                    self._stop_finalize(run_id, "중지 요청 수신")
                    return
                payload = dict(voice_payload or {})
                events = list(payload.get("events", []) or [])
                recognized_text = ""
                for event in events:
                    if not isinstance(event, dict):
                        continue
                    evt_type = str(event.get("type", "") or "").strip().lower()
                    if evt_type == "mic_level":
                        try:
                            mic_level = int(event.get("level", 0) or 0)
                        except Exception:
                            mic_level = 0
                        mic_level = max(0, min(100, int(mic_level)))
                        with self._lock:
                            if int(self._state.get("run_id", -1)) == int(run_id):
                                self._state["voice_mic_level"] = int(mic_level)
                                self._state["updated_at"] = _now_iso_text()
                        continue
                    stage = str(event.get("stage", evt_type) or evt_type).strip().lower()
                    message = str(event.get("message", "") or "").strip()
                    data = event.get("data")
                    level = str(event.get("severity", "info") or "info").strip().lower()
                    if isinstance(data, dict):
                        for key in ("input_text", "stt_text", "recognized_text", "transcript", "text"):
                            raw_value = data.get(key, None)
                            if raw_value is None:
                                continue
                            parsed = str(raw_value or "").strip()
                            if parsed:
                                recognized_text = parsed
                                break
                    if int(streamed_non_mic_events) > 0:
                        continue
                    if data is not None:
                        try:
                            message = f"{message} | {json.dumps(data, ensure_ascii=False)}"
                        except Exception:
                            message = f"{message} | {data}"
                    self._append_log(stage or "voice", message or "-", level=level)
                    if evt_type == "stage":
                        self._sync_step_from_stage(stage)
                result = payload.get("result", {})
                if not isinstance(result, dict):
                    result = {}
                else:
                    result = dict(result)
                if recognized_text and (not str(result.get("input_text", "") or "").strip()):
                    result["input_text"] = recognized_text

                with self._lock:
                    if int(self._state.get("run_id", -1)) == int(run_id):
                        self._state["last_order_result"] = dict(result)

                if not ok_voice:
                    break

                status = str(result.get("status", "") or "").strip().lower()
                if status == "success":
                    break

                if status == "retry" and attempt < int(max_attempts):
                    self._append_log(
                        "voice_request",
                        f"재입력 필요: 자동 재요청 진행({attempt + 1}/{max_attempts})",
                        level="warning",
                    )
                    current_input_text = ""
                    current_request_stt = True
                    current_audio_bytes = None
                    current_audio_filename = "recording.webm"
                    recommend_hint = str(result.get("recommend_menu_hint", "") or "").strip()
                    current_recommend_menu = recommend_hint or current_recommend_menu
                    self._set_active_step("voice_request")
                    continue
                break

            if not ok_voice:
                fail_text = str(result.get("tts_text", "") or "").strip() or str(voice_msg or "음성주문 실패")
                self._fail(run_id, f"음성주문 처리 실패: {fail_text}")
                return

            status = str(result.get("status", "") or "").strip().lower()
            if status != "success":
                fail_text = str(result.get("tts_text", "") or "").strip() or f"주문처리 상태={status or '-'}"
                if status == "retry":
                    fail_text = f"{fail_text} (재시도 한도 {max_attempts}회)"
                self._fail(run_id, fail_text)
                return

            # TTS 완료 플래그를 받은 뒤 로봇동작으로 넘어간다.
            wait_tts_done_flag = bool(request.get("wait_tts_done_flag", True))
            if wait_tts_done_flag:
                wait_timeout_sec = max(
                    0.5,
                    float(
                        request.get(
                            "tts_done_wait_timeout_sec",
                            os.environ.get("BARTENDER_TTS_DONE_WAIT_TIMEOUT_SEC", "12.0"),
                        )
                        or 12.0
                    ),
                )
                post_delay_sec = max(
                    0.0,
                    float(
                        request.get(
                            "post_tts_robot_delay_sec",
                            os.environ.get("BARTENDER_POST_TTS_ROBOT_DELAY_SEC", "0.3"),
                        )
                        or 0.3
                    ),
                )
                self._append_log("tts", f"TTS 완료 플래그 대기 시작(timeout={wait_timeout_sec:.1f}s)")
                got_tts_done = False
                deadline = time.monotonic() + float(wait_timeout_sec)
                while time.monotonic() < deadline:
                    if self._stop_event.is_set():
                        self._stop_finalize(run_id, "중지 요청 수신")
                        return
                    with self._lock:
                        if int(self._tts_done_run_id) == int(run_id):
                            got_tts_done = True
                            break
                    time.sleep(0.05)
                if got_tts_done:
                    if post_delay_sec > 0.0:
                        self._append_log("tts", f"TTS 완료 후 로봇동작 지연: {post_delay_sec:.1f}s")
                        delay_deadline = time.monotonic() + float(post_delay_sec)
                        while time.monotonic() < delay_deadline:
                            if self._stop_event.is_set():
                                self._stop_finalize(run_id, "중지 요청 수신")
                                return
                            time.sleep(0.05)
                else:
                    self._append_log("tts", "TTS 완료 플래그 대기 시간초과: 로봇동작 계속", level="warning")

            self._set_step_done_if_pending("voice_request")
            self._set_step_done_if_pending("stt")
            self._set_step_done_if_pending("llm")
            self._set_step_done_if_pending("recipe")
            if not self._guard_or_stop(run_id, mode):
                return

            if voice_only:
                self._set_step_done_if_pending("robot_action")
                self._set_step_done_if_pending("vision_check")
                self._set_active_step("done")
                self._succeed(run_id, "음성주문 완료(voice_only)")
                return

            self._set_active_step("robot_action")
            menu_offsets = request.get("menu_offsets")
            motion_speed_percent = request.get("motion_speed_percent")
            ok_action, action_msg = backend.run_bartender_first_ingredient_action(
                dict(result),
                menu_offsets=menu_offsets,
                motion_speed_percent=motion_speed_percent,
                execute_enabled_override=bool(request.get("execute_robot_action", True)),
            )
            if not ok_action:
                self._fail(run_id, f"로봇동작 실패: {action_msg}")
                return
            self._append_log("robot_action", str(action_msg or "로봇동작 완료"))
            self._set_step_done_if_pending("robot_action")

            self._set_active_step("vision_check")
            if not self._guard_or_stop(run_id, mode, require_vision=False):
                return
            self._set_step_done_if_pending("vision_check")

            self._set_active_step("done")
            self._succeed(run_id, "시퀀스 완료")
        except Exception as exc:
            self._fail(run_id, f"시퀀스 예외: {exc}")


class BartenderSequenceApiServer:
    def __init__(self, backend, host: str = "127.0.0.1", port: int = 8765):
        self._backend = backend
        self._host = str(host or "127.0.0.1").strip() or "127.0.0.1"
        self._port = int(port)
        self._server = None
        self._thread = None

    @property
    def host(self):
        return self._host

    @property
    def port(self):
        return self._port

    def start(self):
        if self._server is not None:
            return True, f"이미 실행 중: http://{self._host}:{self._port}"

        backend = self._backend

        class _Handler(BaseHTTPRequestHandler):
            server_version = "BartenderSequenceAPI/1.0"

            def log_message(self, fmt, *args):  # pragma: no cover
                return

            def _write_json(self, payload: dict, status_code: int = 200):
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                try:
                    self.send_response(int(status_code))
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                    # Client disconnected before reading the response body.
                    return

            def _read_json(self):
                try:
                    clen = int(self.headers.get("Content-Length", "0"))
                except Exception:
                    clen = 0
                raw = self.rfile.read(max(0, clen)) if clen > 0 else b""
                if not raw:
                    return {}
                try:
                    decoded = json.loads(raw.decode("utf-8"))
                    return decoded if isinstance(decoded, dict) else {}
                except Exception:
                    return None

            def do_GET(self):
                raw_path = str(self.path or "")
                parsed = urlparse(raw_path)
                path = str(parsed.path or "")
                if path.startswith("/api/health"):
                    snap = backend.get_bartender_sequence_snapshot()
                    self._write_json(
                        {
                            "ok": True,
                            "service": "bartender-sequence-api",
                            "sequence_status": str((snap or {}).get("status", "idle")),
                            "running": bool((snap or {}).get("running", False)),
                        }
                    )
                    return
                if path.startswith("/api/sequence/state"):
                    snap = backend.get_bartender_sequence_snapshot()
                    self._write_json({"ok": True, "snapshot": snap})
                    return
                if path.startswith("/api/sequence/precheck"):
                    mode = "auto"
                    try:
                        query = parse_qs(str(parsed.query or ""))
                        mode = str((query.get("mode", ["auto"]) or ["auto"])[0] or "auto")
                    except Exception:
                        mode = "auto"
                    try:
                        payload = backend.get_bartender_sequence_precheck(mode=mode)
                    except Exception as exc:
                        payload = {
                            "ok": False,
                            "mode": str(mode or "auto"),
                            "ready": False,
                            "system_ready": False,
                            "system_reason": f"사전점검 예외: {exc}",
                            "mode_ready": False,
                            "mode_reason": "사전점검 예외",
                            "checked_at": _now_iso_text(),
                        }
                    self._write_json(payload, status_code=200 if bool(payload.get("ok", False)) else 503)
                    return
                self._write_json({"ok": False, "error": "not_found"}, status_code=404)

            def do_POST(self):
                path = str(self.path or "")
                payload = self._read_json()
                if payload is None:
                    self._write_json({"ok": False, "error": "invalid_json"}, status_code=400)
                    return
                if path == "/api/sequence/start":
                    mode = str(payload.get("mode", "manual") or "manual")
                    req = payload.get("request", {})
                    if not isinstance(req, dict):
                        req = {}
                    if not req:
                        req = {
                            "input_text": str(payload.get("input_text", "") or ""),
                            "recommend_menu": str(payload.get("recommend_menu", "") or ""),
                            "allow_llm": bool(payload.get("allow_llm", True)),
                            "request_stt": bool(payload.get("request_stt", False)),
                            "motion_speed_percent": payload.get("motion_speed_percent", None),
                            "menu_offsets": payload.get("menu_offsets", None),
                            "audio_filename": str(payload.get("audio_filename", "recording.webm") or "recording.webm"),
                            "audio_base64": str(payload.get("audio_base64", "") or ""),
                        }
                    ok, msg, snap = backend.start_bartender_sequence(mode=mode, request=req)
                    self._write_json(
                        {"ok": bool(ok), "message": str(msg or ""), "snapshot": snap},
                        status_code=200 if ok else 409,
                    )
                    return
                if path == "/api/sequence/stop":
                    reason = str(payload.get("reason", "") or "")
                    ok, msg, snap = backend.stop_bartender_sequence(reason=reason)
                    self._write_json(
                        {"ok": bool(ok), "message": str(msg or ""), "snapshot": snap},
                        status_code=200 if ok else 409,
                    )
                    return
                if path == "/api/sequence/reset":
                    reason = str(payload.get("reason", "") or "")
                    ok, msg, snap = backend.reset_bartender_sequence(reason=reason)
                    self._write_json(
                        {"ok": bool(ok), "message": str(msg or ""), "snapshot": snap},
                        status_code=200 if ok else 409,
                    )
                    return
                if path == "/api/sequence/tts_done":
                    run_id = payload.get("run_id", None)
                    try:
                        run_id = None if run_id is None else int(run_id)
                    except Exception:
                        run_id = None
                    ok, msg, snap = backend.notify_bartender_tts_done(run_id=run_id)
                    self._write_json(
                        {"ok": bool(ok), "message": str(msg or ""), "snapshot": snap},
                        status_code=200 if ok else 409,
                    )
                    return
                self._write_json({"ok": False, "error": "not_found"}, status_code=404)

        try:
            self._server = ThreadingHTTPServer((self._host, int(self._port)), _Handler)
        except Exception as exc:
            self._server = None
            return False, f"시퀀스 API 서버 시작 실패: {exc}"
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        return True, f"시퀀스 API 서버 시작: http://{self._host}:{self._port}"

    def stop(self):
        server = self._server
        thread = self._thread
        self._server = None
        self._thread = None
        if server is None:
            return True, "시퀀스 API 서버 미실행"
        try:
            server.shutdown()
            server.server_close()
        except Exception as exc:
            return False, f"시퀀스 API 서버 종료 실패: {exc}"
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        return True, "시퀀스 API 서버 종료"
