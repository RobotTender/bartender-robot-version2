# Voice Order Panel / WEB UI

이 문서는 `bartender-robot` 저장소 내부 소스만 사용해 구성된 음성주문 기능을 설명합니다.

## 목적

- `bartender-robot/src/order_integration` 내부 로직으로 음성주문 분류/레시피 도출 수행
- 외부 저장소 참조 없이 단일 저장소에서 운영
- 최종 사용자용 WEB UI 제공

## 주요 파일

- `/home/up/ros2_ws/src/bartender-robot/src/order_integration/voice_order_pipeline.py`
  - 메뉴 분류/레시피 도출 + stage 이벤트/결과 payload 생성 로직
- `/home/up/ros2_ws/src/bartender-robot/src/order_integration/voice_order_worker.py`
  - 백엔드 요청을 받아 STT/분류를 실행하는 음성 워커
- `/home/up/ros2_ws/src/bartender-robot/src/order_integration/voice_order_route.py`
  - 백엔드에서 음성 워커 subprocess를 호출하는 래퍼
- `/home/up/ros2_ws/src/bartender-robot/src/frontend/user_frontend.py`
  - 최종 사용자용 WEB UI 서버
- `/home/up/ros2_ws/src/bartender-robot/src/frontend/developer_frontend.py`
  - 음성주문 패널(상태/업데이트/WEB UI 링크) 표시

## WEB UI 실행

기본 실행(`run_bartender.py`) 시 `launch/system_launch.py`에서 WEB UI가 함께 기동됩니다.

- 기본 주소: `http://127.0.0.1:8000`
- 브라우저에서 마이크 버튼으로 STT 수집 후 주문 처리 가능

필수 환경변수(`.env`):

- `GOOGLE_API_KEY` 또는 `GEMINI_API_KEY` (Gemini STT)
- `OPENAI_API_KEY` (메뉴 분류 LLM)

필수 파이썬 패키지:

- `google-genai`
- `openai`
- `SpeechRecognition`
- `python-dotenv`

## 제한

- 로봇 명령 실행은 음성주문 경로에서 비활성화
- 브라우저 마이크 지원은 Web Speech API 지원 브라우저에 의존
