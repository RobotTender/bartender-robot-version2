import io
import os
from dataclasses import dataclass
from pathlib import Path


try:
    from dotenv import load_dotenv
except Exception:  # pragma: no cover
    load_dotenv = None


_DOTENV_LOADED = False

DEFAULT_TTS_MODEL = "gpt-4o-mini-tts"
DEFAULT_TTS_VOICE = "nova"
DEFAULT_TTS_SPEED = 1.0


@dataclass
class OpenAITTSResult:
    audio_bytes: bytes
    model: str
    voice: str
    speed: float


def _load_dotenv_once():
    global _DOTENV_LOADED
    if _DOTENV_LOADED:
        return
    _DOTENV_LOADED = True
    if load_dotenv is None:
        return
    project_root = Path(__file__).resolve().parent.parent.parent
    dotenv_path = project_root / ".env"
    try:
        load_dotenv(dotenv_path=dotenv_path, override=False)
    except Exception:
        pass


def _resolve_speed(value) -> float:
    raw = value
    if raw is None:
        raw = os.environ.get("VOICE_ORDER_TTS_SPEED", str(DEFAULT_TTS_SPEED))
    try:
        speed = float(raw)
    except Exception:
        speed = float(DEFAULT_TTS_SPEED)
    return max(0.25, min(4.0, float(speed)))


def synthesize_openai_tts(
    text: str,
    *,
    model: str | None = None,
    voice: str | None = None,
    speed: float | None = None,
    response_format: str = "wav",
) -> OpenAITTSResult:
    _load_dotenv_once()

    input_text = str(text or "").strip()
    if not input_text:
        raise RuntimeError("TTS text is empty.")

    api_key = str(os.environ.get("OPENAI_API_KEY", "") or "").strip()
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY is not set.")

    try:
        from openai import OpenAI
    except Exception as exc:
        raise RuntimeError(f"openai 패키지 로드 실패: {exc}") from exc

    model_name = str(model or os.environ.get("VOICE_ORDER_TTS_MODEL", DEFAULT_TTS_MODEL) or "").strip()
    if not model_name:
        model_name = DEFAULT_TTS_MODEL
    voice_name = str(voice or os.environ.get("VOICE_ORDER_TTS_VOICE", DEFAULT_TTS_VOICE) or "").strip()
    if not voice_name:
        voice_name = DEFAULT_TTS_VOICE
    speed_value = _resolve_speed(speed)

    client = OpenAI(api_key=api_key)
    audio_buffer = io.BytesIO()
    try:
        with client.audio.speech.with_streaming_response.create(
            model=model_name,
            voice=voice_name,
            input=input_text,
            response_format=str(response_format or "wav"),
            speed=float(speed_value),
        ) as response:
            for chunk in response.iter_bytes(chunk_size=4096):
                if chunk:
                    audio_buffer.write(chunk)
    except Exception as exc:
        raise RuntimeError(f"OpenAI TTS 합성 실패: {exc}") from exc

    audio_bytes = audio_buffer.getvalue()
    if not audio_bytes:
        raise RuntimeError("OpenAI TTS 결과 오디오가 비어 있습니다.")

    return OpenAITTSResult(
        audio_bytes=audio_bytes,
        model=model_name,
        voice=voice_name,
        speed=float(speed_value),
    )

