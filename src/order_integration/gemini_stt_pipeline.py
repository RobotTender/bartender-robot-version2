import json
import mimetypes
import os
import time
from dataclasses import dataclass


STT_MODEL = str(os.environ.get("VOICE_ORDER_STT_MODEL", "gemini-flash-latest") or "").strip() or "gemini-flash-latest"
MAX_RETRIES = max(1, int(float(os.environ.get("VOICE_ORDER_STT_RETRIES", "3"))))

PROMPT = """You are an audio order analyzer for a Korean bartender app.
Analyze the provided audio and return strict JSON only with these keys:
{
  "transcript": "recognized Korean text",
  "emotion": "happy|sad|angry|tired|neutral",
  "recommend_menu": "soju|beer|somaek",
  "reason": "short reason in Korean"
}
Rules:
- Output must be valid JSON only. No markdown.
- If speech is unclear, set transcript to an empty string and emotion to neutral.
- Keep reason concise (max 1 sentence).
"""


@dataclass
class STTResult:
    text: str
    emotion: str = ""
    recommend_menu: str = ""
    reason: str = ""


_AUDIO_MIME_OVERRIDES = {
    ".webm": "audio/webm",
    ".mp4": "audio/mp4",
    ".wav": "audio/wav",
}


def _guess_mime_type(filename: str | None) -> str:
    if filename:
        ext = os.path.splitext(str(filename))[-1].lower()
        if ext in _AUDIO_MIME_OVERRIDES:
            return _AUDIO_MIME_OVERRIDES[ext]
    guessed, _ = mimetypes.guess_type(filename or "")
    return guessed or "audio/webm"


def _extract_json(text: str) -> dict:
    content = str(text or "").strip()
    if content.startswith("```json"):
        content = content.replace("```json", "", 1).replace("```", "", 1).strip()
    elif content.startswith("```"):
        content = content.replace("```", "", 2).strip()
    data = json.loads(content)
    if not isinstance(data, dict):
        raise RuntimeError("model response is not json object")
    return data


def _is_retryable_error(exc: Exception) -> bool:
    status_code = getattr(exc, "status_code", None)
    if isinstance(status_code, int) and status_code in (429, 500, 503):
        return True
    message = str(exc).upper()
    return ("RESOURCE_EXHAUSTED" in message) or ("INTERNAL" in message) or ("UNAVAILABLE" in message)


def _load_google_genai():
    try:
        from google import genai
        from google.genai import types
    except Exception as exc:
        raise RuntimeError("google-genai 패키지가 필요합니다. `pip install google-genai` 후 재시도하세요.") from exc
    return genai, types


def _generate_content_with_retry(client, audio_part, types):
    last_exc = None
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            return client.models.generate_content(
                model=STT_MODEL,
                contents=[PROMPT, audio_part],
                config=types.GenerateContentConfig(
                    temperature=0,
                    response_mime_type="application/json",
                ),
            )
        except Exception as exc:
            last_exc = exc
            if (not _is_retryable_error(exc)) or (attempt == MAX_RETRIES):
                raise
            time.sleep(2 ** (attempt - 1))
    if last_exc is not None:
        raise last_exc
    raise RuntimeError("Unknown STT failure")


def transcribe_audio_bytes(
    audio_bytes: bytes,
    *,
    filename: str = "recording.webm",
) -> STTResult:
    api_key = str(os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY") or "").strip()
    if not api_key:
        raise RuntimeError("GOOGLE_API_KEY 또는 GEMINI_API_KEY가 .env에 필요합니다.")
    if not isinstance(audio_bytes, (bytes, bytearray)) or len(audio_bytes) < 512:
        raise RuntimeError("audio payload is too short.")

    genai, types = _load_google_genai()

    client = genai.Client(api_key=api_key)
    mime_type = _guess_mime_type(filename)
    audio_part = types.Part.from_bytes(data=bytes(audio_bytes), mime_type=mime_type)
    response = _generate_content_with_retry(client, audio_part, types)

    raw_text = str(getattr(response, "text", "") or "").strip()
    if not raw_text:
        raise RuntimeError("Gemini STT 응답이 비어 있습니다.")
    payload = _extract_json(raw_text)

    transcript = str(payload.get("transcript", "")).strip()
    emotion = str(payload.get("emotion", "")).strip()
    recommend_menu = str(payload.get("recommend_menu", "")).strip().lower()
    reason = str(payload.get("reason", "")).strip()

    return STTResult(
        text=transcript,
        emotion=emotion,
        recommend_menu=recommend_menu,
        reason=reason,
    )
