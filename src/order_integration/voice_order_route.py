import base64
import json
import os
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path


CURRENT_DIR = Path(__file__).resolve().parent
SRC_ROOT = CURRENT_DIR.parent
PROJECT_ROOT = SRC_ROOT.parent
VOICE_ORDER_WORKER_PATH = SRC_ROOT / "order_integration" / "voice_order_worker.py"

VOICE_ORDER_NON_FATAL_STDERR_MARKERS = (
    "alsa lib",
    "unknown pcm",
    "unable to open slave",
    "jack server is not running",
    "cannot connect to server socket",
    "cannot connect to server request channel",
    "jackshmreadwriteptr",
)


def _is_non_fatal_voice_stderr(line: str) -> bool:
    text = str(line or "").strip().lower()
    if not text:
        return True
    return any(marker in text for marker in VOICE_ORDER_NON_FATAL_STDERR_MARKERS)


def run_voice_order_request(
    *,
    input_text: str = "",
    recommend_menu: str = "",
    allow_llm: bool = True,
    request_stt: bool = False,
    emotion: str = "",
    reason: str = "",
    audio_bytes: bytes | bytearray | None = None,
    audio_filename: str = "recording.webm",
    timeout_sec: float = 40.0,
    event_callback=None,
    cancel_checker=None,
):
    text = str(input_text or "").strip()
    rec = str(recommend_menu or "").strip()
    llm_on = bool(allow_llm)
    has_audio = isinstance(audio_bytes, (bytes, bytearray)) and len(audio_bytes) > 0
    req_stt = bool(request_stt) or (not bool(text)) or bool(has_audio)
    payload_in = {
        "input_text": text,
        "recommend_menu": rec,
        "allow_llm": llm_on,
        "request_stt": req_stt,
        "emotion": str(emotion or "").strip(),
        "reason": str(reason or "").strip(),
    }
    if has_audio:
        payload_in["audio_base64"] = base64.b64encode(bytes(audio_bytes)).decode("ascii")
        payload_in["audio_filename"] = str(audio_filename or "recording.webm")

    if not VOICE_ORDER_WORKER_PATH.is_file():
        payload = {
            "events": [
                {
                    "type": "error",
                    "actor": "voice_backend",
                    "message": f"음성처리 워커 경로 없음: {VOICE_ORDER_WORKER_PATH}",
                }
            ],
            "result": {
                "status": "error",
                "selected_menu": "",
                "selected_menu_label": "",
                "tts_text": "음성처리 워커를 찾지 못했습니다.",
                "llm_text": "음성처리 워커를 찾지 못했습니다.",
                "recipe": {},
                "route": "worker_not_found",
                "llm_used": False,
                "llm_reason": "",
            },
            "ok": False,
        }
        return False, payload, "음성처리 워커를 찾지 못했습니다."

    cmd = [sys.executable, str(VOICE_ORDER_WORKER_PATH)]
    events = []
    result = {}
    done_ok = False
    had_fatal_stderr = False

    def _emit_event(evt):
        if event_callback is None:
            return
        try:
            if isinstance(evt, dict):
                event_callback(dict(evt))
        except Exception:
            pass

    def _handle_stdout_line(line_text: str):
        nonlocal done_ok, result
        line = str(line_text or "").strip()
        if not line:
            return
        try:
            evt = json.loads(line)
        except Exception:
            payload = {"type": "log", "actor": "voice_worker", "message": line}
            events.append(payload)
            _emit_event(payload)
            return
        if not isinstance(evt, dict):
            return
        evt_type = str(evt.get("type", "") or "").strip().lower()
        if evt_type == "result":
            result = dict(evt)
            return
        if evt_type == "done":
            done_ok = bool(evt.get("ok", False))
            return
        payload = dict(evt)
        events.append(payload)
        _emit_event(payload)

    def _handle_stderr_line(line_text: str):
        nonlocal done_ok, had_fatal_stderr
        s = str(line_text or "").strip()
        if not s:
            return
        is_warning = _is_non_fatal_voice_stderr(s)
        payload = {
            "type": "stderr",
            "actor": "voice_worker",
            "message": s,
            "severity": "warning" if is_warning else "error",
        }
        events.append(payload)
        _emit_event(payload)
        if not is_warning:
            had_fatal_stderr = True
            done_ok = False

    try:
        proc = subprocess.Popen(
            cmd,
            cwd=str(PROJECT_ROOT),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            bufsize=1,
            env=dict(os.environ, PYTHONUNBUFFERED="1"),
        )
        if proc.stdin is not None:
            proc.stdin.write(json.dumps(payload_in, ensure_ascii=False))
            proc.stdin.flush()
            proc.stdin.close()

        q = queue.Queue()
        stdout_done = False
        stderr_done = False
        deadline = time.monotonic() + float(timeout_sec)

        def _pump(stream, tag: str):
            try:
                if stream is not None:
                    for line in stream:
                        q.put((tag, line))
            finally:
                q.put((f"{tag}_done", None))

        t_out = threading.Thread(target=_pump, args=(proc.stdout, "stdout"), daemon=True)
        t_err = threading.Thread(target=_pump, args=(proc.stderr, "stderr"), daemon=True)
        t_out.start()
        t_err.start()

        rc = None
        while True:
            if callable(cancel_checker):
                try:
                    if bool(cancel_checker()):
                        try:
                            if proc.poll() is None:
                                proc.terminate()
                                proc.wait(timeout=0.4)
                        except Exception:
                            try:
                                proc.kill()
                            except Exception:
                                pass
                        payload = {
                            "events": [
                                {
                                    "type": "warning",
                                    "actor": "voice_backend",
                                    "message": "사용자 중지 요청으로 음성처리를 취소했습니다.",
                                }
                            ],
                            "result": {
                                "status": "canceled",
                                "selected_menu": "",
                                "selected_menu_label": "",
                                "tts_text": "",
                                "llm_text": "",
                                "recipe": {},
                                "route": "worker_canceled",
                                "llm_used": False,
                                "llm_reason": "",
                            },
                            "ok": False,
                        }
                        return False, payload, "사용자 중지 요청으로 취소됨"
                except Exception:
                    pass
            if time.monotonic() > deadline:
                raise subprocess.TimeoutExpired(cmd, timeout=float(timeout_sec))
            try:
                tag, data = q.get(timeout=0.12)
            except queue.Empty:
                if rc is None:
                    rc = proc.poll()
                if rc is not None and stdout_done and stderr_done:
                    break
                continue

            if tag == "stdout":
                _handle_stdout_line(data)
                continue
            if tag == "stderr":
                _handle_stderr_line(data)
                continue
            if tag == "stdout_done":
                stdout_done = True
            elif tag == "stderr_done":
                stderr_done = True

            if rc is None:
                rc = proc.poll()
            if rc is not None and stdout_done and stderr_done:
                break

        if rc is None:
            rc = proc.wait(timeout=0.3)
        rc = int(rc)
        if rc != 0 and not done_ok:
            done_ok = False
    except subprocess.TimeoutExpired:
        try:
            proc.kill()
        except Exception:
            pass
        payload = {
            "events": [{"type": "error", "actor": "voice_backend", "message": f"음성처리 워커 타임아웃({int(timeout_sec)}s)"}],
            "result": {
                "status": "error",
                "selected_menu": "",
                "selected_menu_label": "",
                "tts_text": "음성처리 응답 시간초과",
                "llm_text": "음성처리 응답 시간초과",
                "recipe": {},
                "route": "worker_timeout",
                "llm_used": False,
                "llm_reason": "",
            },
            "ok": False,
        }
        return False, payload, "음성처리 워커 타임아웃"
    except Exception as exc:
        payload = {
            "events": [{"type": "error", "actor": "voice_backend", "message": f"워커 실행 실패: {exc}"}],
            "result": {
                "status": "error",
                "selected_menu": "",
                "selected_menu_label": "",
                "tts_text": f"음성처리 워커 실행 실패: {exc}",
                "llm_text": f"음성처리 워커 실행 실패: {exc}",
                "recipe": {},
                "route": "worker_spawn_error",
                "llm_used": False,
                "llm_reason": "",
            },
            "ok": False,
        }
        return False, payload, f"음성처리 워커 실행 실패: {exc}"

    if (not had_fatal_stderr) and (not done_ok) and rc == 0:
        done_ok = True

    if not result:
        result = {
            "status": "error" if not done_ok else "unknown",
            "selected_menu": "",
            "selected_menu_label": "",
            "tts_text": "음성처리 결과 없음" if not done_ok else "-",
            "llm_text": "음성처리 결과 없음" if not done_ok else "-",
            "recipe": {},
            "route": "worker_no_result",
            "llm_used": False,
            "llm_reason": "",
        }

    payload = {
        "events": events,
        "result": result,
        "ok": bool(done_ok),
    }
    if done_ok:
        return True, payload, "음성처리 결과 수신 완료"
    return False, payload, "음성처리 실패"
