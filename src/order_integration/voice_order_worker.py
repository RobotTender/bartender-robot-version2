#!/usr/bin/env python3
import base64
import json
import os
import sys
import time
from pathlib import Path
import numpy as np


CURRENT_DIR = Path(__file__).resolve().parent
SRC_ROOT = CURRENT_DIR.parent
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

try:
    from dotenv import load_dotenv
except Exception:  # pragma: no cover - optional dependency
    load_dotenv = None

from order_integration.voice_order_pipeline import build_voice_order_runtime
from order_integration.gemini_stt_pipeline import transcribe_audio_bytes


def _env_float(name: str, default: float, min_value: float | None = None) -> float:
    raw = str(os.environ.get(name, "") or "").strip()
    if not raw:
        value = float(default)
    else:
        try:
            value = float(raw)
        except Exception:
            value = float(default)
    if min_value is not None:
        value = max(float(min_value), float(value))
    return float(value)


def _env_bool(name: str, default: bool = False) -> bool:
    raw = str(os.environ.get(name, "") or "").strip().lower()
    if raw in ("1", "true", "yes", "on"):
        return True
    if raw in ("0", "false", "no", "off"):
        return False
    return bool(default)


def _select_microphone_device(sr_module):
    try:
        names = list(sr_module.Microphone.list_microphone_names() or [])
    except Exception:
        names = []

    raw_index = str(os.environ.get("VOICE_ORDER_MIC_DEVICE_INDEX", "") or "").strip()
    if raw_index:
        try:
            idx = int(raw_index)
            if 0 <= idx < len(names):
                name = str(names[idx] or f"index:{idx}") if names else f"index:{idx}"
                return idx, str(name), "env_index"
        except Exception:
            pass

    hint = str(os.environ.get("VOICE_ORDER_MIC_DEVICE_HINT", "") or "").strip().lower()
    if hint:
        for idx, name in enumerate(names):
            text = str(name or "").lower()
            if hint in text:
                return idx, str(name), "env_hint"

    if _env_bool("VOICE_ORDER_MIC_AUTO_SCAN", False):
        if names:
            for token in ("pipewire", "pulse", "usb", "mic", "default"):
                for idx, name in enumerate(names):
                    text = str(name or "").lower()
                    if token in text:
                        return idx, str(name), f"auto:{token}"

    return None, "default", "default"


def _emit(event_type: str, **payload):
    body = {"type": str(event_type), **payload}
    print(json.dumps(body, ensure_ascii=False), flush=True)


def _read_payload() -> dict:
    raw = (sys.stdin.read() or "").strip()
    if not raw:
        return {}
    try:
        return json.loads(raw)
    except Exception:
        # line-oriented fallback
        first_line = raw.splitlines()[0].strip()
        if not first_line:
            return {}
        return json.loads(first_line)


def _capture_stt_payload(audio_bytes=None, audio_filename: str = "recording.wav"):
    events = []

    def _push_event(body: dict):
        payload = dict(body or {})
        events.append(payload)
        evt_type = str(payload.get("type", "stage") or "stage").strip() or "stage"
        emit_body = dict(payload)
        emit_body.pop("type", None)
        _emit(evt_type, **emit_body)

    def _stage(stage: str, actor: str, message: str, data=None):
        body = {
            "type": "stage",
            "stage": str(stage),
            "actor": str(actor),
            "message": str(message),
        }
        if data is not None:
            body["data"] = data
        _push_event(body)

    def _calc_level_from_pcm16(raw_bytes) -> int:
        if not isinstance(raw_bytes, (bytes, bytearray)) or len(raw_bytes) <= 0:
            return 0
        try:
            samples = np.frombuffer(bytes(raw_bytes), dtype=np.int16)
            if samples.size <= 0:
                rms = 0.0
            else:
                rms = float(np.sqrt(np.mean(np.square(samples.astype(np.float32)))))
        except Exception:
            rms = 0.0
        if rms <= 0.0:
            return 0
        normalized = min(1.0, float(rms) / float(level_ref))
        level = int(max(0, min(100, round((normalized ** 0.65) * 100.0))))
        if int(level) < int(level_gate):
            return 0
        return int(level)

    _stage("input_request", "voice_processor", "백엔드 음성 입력 요청 수신", {"request_stt": True})

    uploaded_audio = bytes(audio_bytes) if isinstance(audio_bytes, (bytes, bytearray)) else b""
    try:
        if uploaded_audio:
            _stage(
                "stt_wait",
                "stt_pipeline",
                "오디오 입력 수신",
                {"source": "webui", "bytes": len(uploaded_audio)},
            )
            _stage("stt_process", "stt_pipeline", "Gemini STT 처리 중")
            stt_result = transcribe_audio_bytes(uploaded_audio, filename=str(audio_filename or "recording.webm"))
        else:
            try:
                import speech_recognition as sr
            except Exception as e:
                _push_event({"type": "error", "actor": "stt_pipeline", "message": f"speech_recognition 미설치: {e}"})
                return False, {}, events, "speech_recognition 미설치"
            recognizer = sr.Recognizer()
            listen_timeout = _env_float("VOICE_ORDER_MIC_LISTEN_TIMEOUT_SEC", 3.5, min_value=1.0)
            phrase_limit = _env_float("VOICE_ORDER_MIC_PHRASE_TIME_LIMIT_SEC", 4.0, min_value=1.0)
            ambient_sec = _env_float("VOICE_ORDER_MIC_AMBIENT_SEC", 0.12, min_value=0.0)
            pre_listen_delay_sec = _env_float("VOICE_ORDER_STT_PRE_LISTEN_DELAY_SEC", 0.13, min_value=0.0)
            min_voice_sec = _env_float("VOICE_ORDER_MIN_VOICE_SEC", 0.1, min_value=0.1)
            max_short_retry = int(max(0.0, min(8.0, _env_float("VOICE_ORDER_SHORT_RETRY_COUNT", 0.0, min_value=0.0))))
            level_gate = int(max(0.0, min(60.0, _env_float("VOICE_ORDER_MIC_LEVEL_GATE", 8.0, min_value=0.0))))
            level_ref = _env_float("VOICE_ORDER_MIC_LEVEL_REF", 3000.0, min_value=400.0)
            recognizer.pause_threshold = _env_float("VOICE_ORDER_MIC_PAUSE_SEC", 0.45, min_value=0.1)
            recognizer.non_speaking_duration = _env_float("VOICE_ORDER_MIC_NON_SPEAKING_SEC", 0.2, min_value=0.05)
            recognizer.phrase_threshold = _env_float("VOICE_ORDER_MIC_PHRASE_THRESHOLD_SEC", 0.2, min_value=0.05)
            mic_index, mic_name, mic_source = _select_microphone_device(sr)
            mic_attempts = [(mic_index, mic_name, mic_source)]
            if mic_index is not None and "default" not in str(mic_source).lower():
                # 선택 장치에서 timeout/열기실패가 나면 시스템 default로 한 번 더 재시도한다.
                mic_attempts.append((None, "default", "fallback_default"))

            audio = None
            last_mic_err = None
            total_attempts = max(1, len(mic_attempts))
            for attempt_idx, (attempt_index, attempt_name, attempt_source) in enumerate(mic_attempts, start=1):
                _stage(
                    "stt_open",
                    "stt_pipeline",
                    "마이크 장치 준비",
                    {
                        "attempt": int(attempt_idx),
                        "attempt_total": int(total_attempts),
                        "device_index": attempt_index,
                        "device_name": attempt_name,
                        "device_select": attempt_source,
                        "listen_timeout_sec": listen_timeout,
                        "phrase_time_limit_sec": phrase_limit,
                    },
                )
                mic_kwargs = {}
                if attempt_index is not None:
                    mic_kwargs["device_index"] = int(attempt_index)
                try:
                    with sr.Microphone(**mic_kwargs) as source:
                        if ambient_sec > 0.0:
                            recognizer.adjust_for_ambient_noise(source, duration=float(ambient_sec))
                        # listen() 내부 read를 가로채 mic_level(0~100) 이벤트를 전달한다.
                        read_wrapped = False
                        last_level_emit_at = 0.0

                        def _emit_mic_level(raw_bytes):
                            nonlocal last_level_emit_at
                            now = time.monotonic()
                            if (now - float(last_level_emit_at)) < 0.08:
                                return
                            last_level_emit_at = now
                            level = _calc_level_from_pcm16(raw_bytes)
                            _push_event(
                                {
                                    "type": "mic_level",
                                    "actor": "stt_pipeline",
                                    "level": int(level),
                                }
                            )

                        stream_orig = None
                        try:
                            stream_orig = getattr(source, "stream", None)
                            if stream_orig is not None:
                                class _StreamProxy:
                                    def __init__(self, inner):
                                        self._inner = inner

                                    def read(self, *args, **kwargs):
                                        chunk = self._inner.read(*args, **kwargs)
                                        _emit_mic_level(chunk)
                                        return chunk

                                    def __getattr__(self, name):
                                        return getattr(self._inner, name)

                                setattr(source, "stream", _StreamProxy(stream_orig))
                                read_wrapped = True
                        except Exception:
                            read_wrapped = False
                        if not read_wrapped:
                            _stage(
                                "stt_open",
                                "stt_pipeline",
                                "마이크 레벨 실시간 수집 비활성(드라이버 제한)",
                                {"realtime_level": False},
                            )
                        if float(pre_listen_delay_sec) > 0.0:
                            time.sleep(float(pre_listen_delay_sec))
                        _stage(
                            "stt_wait",
                            "stt_pipeline",
                            "마이크 입력 대기(말씀하세요)",
                            {
                                "attempt": int(attempt_idx),
                                "attempt_total": int(total_attempts),
                                "device_index": attempt_index,
                                "device_name": attempt_name,
                                "device_select": attempt_source,
                                "listen_timeout_sec": listen_timeout,
                                "phrase_time_limit_sec": phrase_limit,
                                "pre_listen_delay_sec": pre_listen_delay_sec,
                                "min_voice_sec": min_voice_sec,
                            },
                        )
                        short_retry_count = 0
                        try:
                            while True:
                                audio = recognizer.listen(
                                    source,
                                    timeout=float(listen_timeout),
                                    phrase_time_limit=float(phrase_limit),
                                )
                                if (not read_wrapped) and audio is not None:
                                    try:
                                        fallback_level = _calc_level_from_pcm16(
                                            audio.get_raw_data(convert_rate=16000, convert_width=2)
                                        )
                                        _push_event(
                                            {
                                                "type": "mic_level",
                                                "actor": "stt_pipeline",
                                                "level": int(fallback_level),
                                            }
                                        )
                                    except Exception:
                                        pass

                                # 말소리로 보기 어려운 아주 짧은 입력(잡음)은 무시하고 재대기한다.
                                frame_data = getattr(audio, "frame_data", b"") if audio is not None else b""
                                sample_rate = int(getattr(audio, "sample_rate", 16000) or 16000) if audio is not None else 16000
                                sample_width = int(getattr(audio, "sample_width", 2) or 2) if audio is not None else 2
                                if sample_rate <= 0:
                                    sample_rate = 16000
                                if sample_width <= 0:
                                    sample_width = 2
                                captured_sec = float(len(frame_data)) / float(sample_rate * sample_width)
                                if captured_sec < float(min_voice_sec) and short_retry_count < int(max_short_retry):
                                    short_retry_count += 1
                                    _stage(
                                        "stt_wait",
                                        "stt_pipeline",
                                        "짧은 입력 감지, 다시 말씀해 주세요",
                                        {
                                            "captured_sec": round(float(captured_sec), 3),
                                            "min_voice_sec": float(min_voice_sec),
                                            "short_retry": int(short_retry_count),
                                            "short_retry_max": int(max_short_retry),
                                        },
                                    )
                                    continue
                                break
                        finally:
                            if read_wrapped:
                                try:
                                    setattr(source, "stream", stream_orig)
                                except Exception:
                                    pass
                    break
                except (sr.WaitTimeoutError, OSError) as mic_err:
                    last_mic_err = mic_err
                    retry_allowed = isinstance(mic_err, (OSError, sr.WaitTimeoutError))
                    if retry_allowed and attempt_idx < total_attempts:
                        _stage(
                            "stt_retry",
                            "stt_pipeline",
                            "마이크 장치 재시도",
                            {
                                "reason": str(mic_err),
                                "next_attempt": int(attempt_idx + 1),
                            },
                        )
                        continue
                    raise
            if audio is None and last_mic_err is not None:
                raise last_mic_err
            _stage("stt_process", "stt_pipeline", "Gemini STT 처리 중")
            wav_bytes = audio.get_wav_data(convert_rate=16000, convert_width=2)
            stt_result = transcribe_audio_bytes(wav_bytes, filename="recording.wav")
        text = str(stt_result.text or "").strip()
        if not text:
            _push_event({"type": "error", "actor": "stt_pipeline", "message": "STT 결과가 비어 있습니다."})
            return False, {}, events, "STT 결과가 비어 있습니다."
        meta = {
            "stt_text": text,
            "emotion": str(stt_result.emotion or "").strip() or "neutral",
            "recommend_menu": str(stt_result.recommend_menu or "").strip(),
            "reason": str(stt_result.reason or "").strip(),
        }
        _stage("stt", "stt_pipeline", "Gemini STT 결과 반영", meta)
        return (
            True,
            {
                "text": text,
                "emotion": meta["emotion"],
                "recommend_menu": meta["recommend_menu"],
                "reason": meta["reason"],
            },
            events,
            "",
        )
    except Exception as e:
        # Keep speech_recognition-specific exceptions only when microphone mode was used.
        if not uploaded_audio:
            try:
                import speech_recognition as sr
                if isinstance(e, OSError):
                    msg = f"마이크 장치 열기 실패: {e}"
                    _push_event({"type": "error", "actor": "stt_pipeline", "message": msg})
                    return False, {}, events, msg
                if isinstance(e, sr.WaitTimeoutError):
                    _push_event({"type": "error", "actor": "stt_pipeline", "message": "마이크 입력 시간초과"})
                    return False, {}, events, "마이크 입력 시간초과"
                if isinstance(e, sr.UnknownValueError):
                    _push_event({"type": "error", "actor": "stt_pipeline", "message": "STT 인식 실패"})
                    return False, {}, events, "STT 인식 실패"
                if isinstance(e, sr.RequestError):
                    msg = f"STT 서비스 요청 실패: {e}"
                    _push_event({"type": "error", "actor": "stt_pipeline", "message": msg})
                    return False, {}, events, msg
            except Exception:
                pass
        msg = f"Gemini STT 처리 예외: {e}"
        _push_event({"type": "error", "actor": "stt_pipeline", "message": msg})
        return False, {}, events, msg


def main() -> int:
    if load_dotenv is not None:
        project_root = SRC_ROOT.parent
        load_dotenv(dotenv_path=project_root / ".env", override=False)

    payload = _read_payload()
    input_text = str(payload.get("input_text", "") or "").strip()
    recommend_menu = str(payload.get("recommend_menu", "") or "").strip()
    allow_llm = bool(payload.get("allow_llm", True))
    emotion = str(payload.get("emotion", "") or "").strip()
    reason = str(payload.get("reason", "") or "").strip()
    audio_filename = str(payload.get("audio_filename", "recording.webm") or "recording.webm").strip() or "recording.webm"
    audio_bytes = b""
    audio_base64 = str(payload.get("audio_base64", "") or "").strip()
    if audio_base64:
        try:
            audio_bytes = base64.b64decode(audio_base64, validate=True)
        except Exception as e:
            _emit("error", actor="voice_worker", message=f"오디오 payload 디코딩 실패: {e}")
            _emit(
                "result",
                worker_pid=os.getpid(),
                status="error",
                selected_menu="",
                selected_menu_label="",
                tts_text="오디오 입력 디코딩 실패",
                llm_text="오디오 입력 디코딩 실패",
                recipe={},
                route="audio_payload_decode_error",
                llm_used=False,
                llm_reason="",
            )
            _emit("done", ok=False)
            return 1
    request_stt = bool(payload.get("request_stt", False)) or (not bool(input_text)) or bool(audio_bytes)

    if request_stt:
        ok_stt, stt_payload, _stt_events, stt_msg = _capture_stt_payload(
            audio_bytes=audio_bytes if audio_bytes else None,
            audio_filename=audio_filename,
        )
        if not ok_stt:
            _emit(
                "result",
                worker_pid=os.getpid(),
                status="error",
                selected_menu="",
                selected_menu_label="",
                tts_text=str(stt_msg or "STT 실패"),
                llm_text=str(stt_msg or "STT 실패"),
                recipe={},
                route="stt_error",
                llm_used=False,
                llm_reason="",
            )
            _emit("done", ok=False)
            return 1
        input_text = str((stt_payload or {}).get("text", "") or "").strip()
        if not recommend_menu:
            recommend_menu = str((stt_payload or {}).get("recommend_menu", "") or "").strip()
        emotion = str((stt_payload or {}).get("emotion", "") or "").strip()
        reason = str((stt_payload or {}).get("reason", "") or "").strip()

    output = build_voice_order_runtime(
        input_text=input_text,
        recommend_menu=recommend_menu,
        allow_llm=allow_llm,
        emotion=emotion,
        reason=reason,
    )
    for event in output.events:
        body = dict(event)
        evt_type = str(body.pop("type", "stage"))
        _emit(evt_type, **body)
    _emit("result", worker_pid=os.getpid(), **dict(output.result_payload))
    _emit("done", ok=bool(output.done_ok))
    return 0 if bool(output.done_ok) else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # pragma: no cover - worker crash fallback
        _emit("error", actor="voice_worker", message=f"워커 예외 발생: {exc}")
        _emit("done", ok=False)
        raise
