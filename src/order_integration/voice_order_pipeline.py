import os
from dataclasses import dataclass


MODEL = os.environ.get("VOICE_ORDER_MODEL", "gpt-5-nano")

MENU_LABELS = {
    "soju": "소주",
    "beer": "맥주",
    "somaek": "소맥",
}

MENU_ALIASES = {
    "soju": ("소주",),
    "beer": ("맥주", "비어"),
    "somaek": ("소맥", "소주맥주", "소주 맥주"),
}

RECIPE_BY_MENU = {
    "soju": {"soju": 50},
    "beer": {"beer": 200},
    "somaek": {"soju": 60, "beer": 140},
}

ORDER_CUES = ("줘", "주세요", "주문", "말아", "말아줘", "한잔", "한 잔", "주라", "내놔")
CONFIRM_CUES = ("응", "어", "그래", "좋아", "맞아", "그걸로", "그거", "할게", "할게요", "부탁")
REJECT_CUES = ("아니", "말고", "싫어", "괜찮아", "됐어")
NEGATIVE_CONFIRM_BLOCKERS = (
    "안 좋아",
    "안좋아",
    "좋지 않아",
    "좋지않아",
    "기분이 안 좋아",
    "기분이 안좋아",
)


@dataclass
class VoiceOrderDecision:
    input_text: str
    status: str
    selected_menu: str
    tts_text: str
    recipe: dict
    route: str
    used_llm: bool
    llm_reason: str


@dataclass
class VoiceOrderRuntimeOutput:
    events: list[dict]
    result_payload: dict
    done_ok: bool


def detect_menu_from_text(text: str) -> str:
    normalized = str(text or "").strip()
    if not normalized:
        return ""
    for menu_code, aliases in MENU_ALIASES.items():
        if any(alias in normalized for alias in aliases):
            return menu_code
    return ""


def build_order_confirmation_text(selected_menu: str) -> str:
    menu_label = MENU_LABELS.get(selected_menu, "해당 메뉴")
    return f"{menu_label} 주문 확인했습니다. 제조시작합니다."


def build_retry_prompt_text() -> str:
    return "어떤 것을 원하시나요? 저희는 소주, 맥주, 소맥이 준비되어 있습니다."


def build_recipe(selected_menu: str) -> dict:
    return dict(RECIPE_BY_MENU.get(str(selected_menu or ""), {}))


def _extract_response_text(response) -> str:
    text = str(getattr(response, "output_text", "") or "").strip()
    if text:
        return text
    for item in getattr(response, "output", []) or []:
        for content in getattr(item, "content", []) or []:
            content_type = str(getattr(content, "type", "") or "")
            if content_type in ("output_text", "text"):
                candidate = str(getattr(content, "text", "") or "").strip()
                if candidate:
                    return candidate
    return ""


def _resolve_menu_with_llm(input_text: str, recommend_menu: str = "") -> tuple[str, str]:
    api_key = str(os.environ.get("OPENAI_API_KEY") or "").strip()
    if not api_key:
        return "", "OPENAI_API_KEY 없음"
    try:
        from openai import OpenAI
    except Exception:
        return "", "openai 패키지 없음"

    prompt = (
        "다음 한국어 발화에서 주문 메뉴를 분류하세요.\n"
        f"발화: {input_text}\n"
        f"이전 추천 메뉴: {recommend_menu}\n"
        f"선택지: {', '.join(MENU_LABELS.keys())}\n"
        "반드시 선택지 중 하나 또는 빈 문자열만 답하세요."
    )
    try:
        client = OpenAI(api_key=api_key)
        response = client.responses.create(model=MODEL, input=prompt)
        raw = _extract_response_text(response).strip().lower()
    except Exception as exc:
        return "", f"LLM 호출 실패: {exc}"

    if raw in MENU_LABELS:
        return raw, "LLM 매칭 성공"
    detected = detect_menu_from_text(raw)
    if detected:
        return detected, "LLM 응답 별칭 매칭"
    return "", f"LLM 판별 불가: {raw}"


def classify_voice_order(
    input_text: str,
    *,
    recommend_menu: str = "",
    allow_llm: bool = True,
) -> VoiceOrderDecision:
    text = str(input_text or "").strip()
    if not text:
        return VoiceOrderDecision(
            input_text=text,
            status="error",
            selected_menu="",
            tts_text="입력이 비어 있습니다.",
            recipe={},
            route="empty_input",
            used_llm=False,
            llm_reason="",
        )

    selected_menu = detect_menu_from_text(text)
    has_confirm_cue = any(cue in text for cue in CONFIRM_CUES)
    if has_confirm_cue and any(blocker in text for blocker in NEGATIVE_CONFIRM_BLOCKERS):
        has_confirm_cue = False
    has_reject_cue = any(cue in text for cue in REJECT_CUES)

    if has_reject_cue and not selected_menu:
        return VoiceOrderDecision(
            input_text=text,
            status="retry",
            selected_menu="",
            tts_text=build_retry_prompt_text(),
            recipe={},
            route="reject_without_menu",
            used_llm=False,
            llm_reason="",
        )

    if selected_menu:
        return VoiceOrderDecision(
            input_text=text,
            status="success",
            selected_menu=selected_menu,
            tts_text=build_order_confirmation_text(selected_menu),
            recipe=build_recipe(selected_menu),
            route="direct_keyword",
            used_llm=False,
            llm_reason="",
        )

    normalized_recommend = str(recommend_menu or "").strip().lower()
    if has_confirm_cue and normalized_recommend in MENU_LABELS:
        return VoiceOrderDecision(
            input_text=text,
            status="success",
            selected_menu=normalized_recommend,
            tts_text=build_order_confirmation_text(normalized_recommend),
            recipe=build_recipe(normalized_recommend),
            route="confirm_recommendation",
            used_llm=False,
            llm_reason="",
        )

    # STT 메타로 추천 메뉴가 이미 존재하는 경우:
    # 사용자가 명시적으로 확정/거절/직접메뉴지정을 하기 전에는
    # success로 확정하지 않고 재질문(retry)로 유지한다.
    if normalized_recommend in MENU_LABELS:
        return VoiceOrderDecision(
            input_text=text,
            status="retry",
            selected_menu="",
            tts_text=build_retry_prompt_text(),
            recipe={},
            route="recommendation_pending_confirmation",
            used_llm=False,
            llm_reason="",
        )

    if allow_llm:
        resolved_menu, reason = _resolve_menu_with_llm(text, recommend_menu=normalized_recommend)
        if resolved_menu:
            return VoiceOrderDecision(
                input_text=text,
                status="success",
                selected_menu=resolved_menu,
                tts_text=build_order_confirmation_text(resolved_menu),
                recipe=build_recipe(resolved_menu),
                route="llm_match",
                used_llm=True,
                llm_reason=reason,
            )
        return VoiceOrderDecision(
            input_text=text,
            status="retry",
            selected_menu="",
            tts_text=build_retry_prompt_text(),
            recipe={},
            route="llm_no_match",
            used_llm=True,
            llm_reason=reason,
        )

    return VoiceOrderDecision(
        input_text=text,
        status="retry",
        selected_menu="",
        tts_text=build_retry_prompt_text(),
        recipe={},
        route="unresolved_without_llm",
        used_llm=False,
        llm_reason="",
    )


def build_voice_order_runtime(
    input_text: str,
    *,
    recommend_menu: str = "",
    allow_llm: bool = True,
    emotion: str = "",
    reason: str = "",
) -> VoiceOrderRuntimeOutput:
    text = str(input_text or "").strip()
    rec_menu = str(recommend_menu or "").strip()
    llm_on = bool(allow_llm)
    emotion_text = str(emotion or "").strip()
    reason_text = str(reason or "").strip()

    events: list[dict] = []

    def _stage(stage: str, actor: str, message: str, data=None):
        payload = {
            "type": "stage",
            "stage": str(stage),
            "actor": str(actor),
            "message": str(message),
        }
        if data is not None:
            payload["data"] = data
        events.append(payload)

    _stage(
        "input",
        "frontend",
        "입력 수신",
        {"input_text": text},
    )
    _stage(
        "stt",
        "stt_pipeline",
        "실시간 STT 결과 반영",
        {"stt_text": text},
    )
    if emotion_text or rec_menu or reason_text:
        _stage(
            "stt_meta",
            "stt_pipeline",
            "STT 메타데이터 반영",
            {
                "emotion": emotion_text or "neutral",
                "recommend_menu": rec_menu,
                "reason": reason_text,
            },
        )

    decision = classify_voice_order(text, recommend_menu=rec_menu, allow_llm=llm_on)

    _stage(
        "classify",
        "order_classifier",
        "주문 텍스트 분류 완료",
        {
            "status": decision.status,
            "route": decision.route,
            "selected_menu": decision.selected_menu,
        },
    )

    if decision.used_llm:
        _stage(
            "llm",
            "order_llm",
            "LLM 판별 단계 수행",
            {"reason": decision.llm_reason},
        )

    if decision.status == "success" and decision.selected_menu:
        _stage(
            "recipe",
            "menu_detail",
            "레시피 도출 완료",
            {"selected_menu": decision.selected_menu, "recipe": decision.recipe},
        )
    elif decision.status == "retry":
        _stage(
            "retry",
            "menu_detail",
            "메뉴 재확인 필요: 재입력 대기",
            {"recommend_menu": rec_menu, "reason": reason_text},
        )

    # 대화형 응답 보강:
    # retry 상황에서는 STT 메타(reason/recommend_menu)를 TTS에 포함해
    # "추천 이유 + 재질문" 형태로 안내한다.
    spoken_tts_text = str(decision.tts_text or "").strip()
    if decision.status == "retry":
        tts_chunks = []
        if reason_text:
            tts_chunks.append(str(reason_text))
        if rec_menu in MENU_LABELS:
            tts_chunks.append(f"추천은 {MENU_LABELS.get(rec_menu, rec_menu)}입니다.")
        if spoken_tts_text:
            tts_chunks.append(spoken_tts_text)
        merged = " ".join(chunk for chunk in tts_chunks if str(chunk).strip())
        if merged:
            spoken_tts_text = merged

    result_payload = {
        "status": decision.status,
        "selected_menu": decision.selected_menu,
        "selected_menu_label": MENU_LABELS.get(decision.selected_menu, ""),
        "tts_text": spoken_tts_text,
        "llm_text": spoken_tts_text,
        "recipe": decision.recipe,
        "route": decision.route,
        "llm_used": bool(decision.used_llm),
        "llm_reason": str(decision.llm_reason or ""),
        "emotion": emotion_text,
        "recommend_menu_hint": rec_menu,
        "reason": reason_text,
    }
    done_ok = decision.status != "error"
    return VoiceOrderRuntimeOutput(events=events, result_payload=result_payload, done_ok=done_ok)
