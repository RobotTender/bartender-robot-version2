# Voice Order Panel / Web UI

Last updated: 2026-03-26

## English

### Purpose

- Run voice ordering from this repository only.
- Support developer debugging and end-user browser flow.

### Core files

- `src/order_integration/voice_order_pipeline.py`
- `src/order_integration/voice_order_worker.py`
- `src/order_integration/voice_order_route.py`
- `src/frontend/developer_frontend.py`
- `src/frontend/user_frontend.py`

### Run

`run_bartender.py` starts web UI process by default via `system_launch.py`.

Typical URL:

- `http://127.0.0.1:8000`

Order entry gate:

- `USER_FRONTEND_ENABLED=1` (legacy: `VOICE_ORDER_WEBUI_ENABLED=1`)

### Dependencies

- `google-genai`
- `openai`
- `SpeechRecognition`
- `python-dotenv`

### Limitations

- Browser mic behavior depends on browser/device support.
- Robot action execution is controlled by backend sequence flow, not directly by worker.

## Korean (한국어)

### 목적

- 이 저장소 내부 코드만으로 음성주문을 실행합니다.
- 개발자 디버그와 최종 사용자 브라우저 흐름을 함께 지원합니다.

### 핵심 파일

- `src/order_integration/voice_order_pipeline.py`
- `src/order_integration/voice_order_worker.py`
- `src/order_integration/voice_order_route.py`
- `src/frontend/developer_frontend.py`
- `src/frontend/user_frontend.py`

### 실행

`run_bartender.py` 실행 시 `system_launch.py` 경로에서 웹 UI 프로세스가 기본 기동됩니다.

기본 접속 주소:

- `http://127.0.0.1:8000`

주문 진입 활성화:

- `USER_FRONTEND_ENABLED=1` (하위호환: `VOICE_ORDER_WEBUI_ENABLED=1`)

### 의존 패키지

- `google-genai`
- `openai`
- `SpeechRecognition`
- `python-dotenv`

### 제한사항

- 브라우저 마이크 동작은 브라우저/장치 지원 여부에 영향을 받습니다.
- 로봇 동작 실행 권한은 워커가 아니라 백엔드 시퀀스 흐름에서 제어됩니다.
