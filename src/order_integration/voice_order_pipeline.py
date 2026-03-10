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
