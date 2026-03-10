#!/usr/bin/env python3
import json
import os
import sys
from pathlib import Path


CURRENT_DIR = Path(__file__).resolve().parent
SRC_ROOT = CURRENT_DIR.parent
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

try:
    from dotenv import load_dotenv
except Exception:  # pragma: no cover - optional dependency
    load_dotenv = None

from order_integration.voice_order_pipeline import MENU_LABELS, classify_voice_order


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


def main() -> int:
    if load_dotenv is not None:
        project_root = SRC_ROOT.parent
        load_dotenv(dotenv_path=project_root / ".env", override=False)

    payload = _read_payload()
    input_text = str(payload.get("input_text", "") or "").strip()
    recommend_menu = str(payload.get("recommend_menu", "") or "").strip()
    allow_llm = bool(payload.get("allow_llm", True))

    _emit(
        "stage",
        stage="input",
        actor="frontend",
        message="테스트 입력 수신",
        data={"input_text": input_text},
    )
    _emit(
        "stage",
        stage="stt",
        actor="stt_pipeline",
        message="테스트 모드: 실시간 음성수집 비활성화, 입력 텍스트를 STT 결과로 사용",
    )

    decision = classify_voice_order(input_text, recommend_menu=recommend_menu, allow_llm=allow_llm)
    _emit(
        "stage",
        stage="classify",
        actor="order_classifier",
        message="주문 텍스트 분류 완료",
        data={
            "status": decision.status,
            "route": decision.route,
            "selected_menu": decision.selected_menu,
        },
    )

    if decision.used_llm:
        _emit(
            "stage",
            stage="llm",
            actor="order_llm",
            message="LLM 판별 단계 수행",
            data={"reason": decision.llm_reason},
        )

    if decision.selected_menu:
        _emit(
            "stage",
            stage="recipe",
            actor="menu_detail",
            message="레시피 도출 완료",
            data={"selected_menu": decision.selected_menu, "recipe": decision.recipe},
        )
    else:
        _emit(
            "stage",
            stage="recipe",
            actor="menu_detail",
            message="선택된 메뉴가 없어 레시피를 도출하지 못했습니다.",
            data={},
        )

    _emit(
        "stage",
        stage="robot",
        actor="robot_command",
        message="로봇 동작은 테스트 모드에서 비활성화됨",
        data={"robot_action_enabled": False},
    )

    _emit(
        "stage",
        stage="html",
        actor="order_ui",
        message="HTML 주문 UI 연동은 현재 비활성화 상태",
        data={"html_ui_enabled": False},
    )

    _emit(
        "result",
        status=decision.status,
        selected_menu=decision.selected_menu,
        selected_menu_label=MENU_LABELS.get(decision.selected_menu, ""),
        tts_text=decision.tts_text,
        recipe=decision.recipe,
        route=decision.route,
        robot_action_enabled=False,
        html_ui_enabled=False,
        worker_pid=os.getpid(),
    )
    _emit("done", ok=(decision.status != "error"))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # pragma: no cover - worker crash fallback
        _emit("error", actor="voice_worker", message=f"워커 예외 발생: {exc}")
        _emit("done", ok=False)
        raise

