# Voice Order Test Panel

이 문서는 `bartender-robot` UI에 추가된 좌측 `음성주문 LLM 테스트` 섹션 설명입니다.

## 목적

- `robot-llm-combine` 주문 파이프라인의 핵심 흐름(입력 -> 분류 -> 레시피)을 GUI 안에서 점검
- 로봇 동작 호출 없이, 텍스트 기반 테스트만 수행
- 별도 프로세스로 실행하여 메인 UI와 분리

## 추가된 파일

- `/home/up/ros2_ws/src/bartender-robot/src/order_integration/voice_order_pipeline.py`
  - 메뉴 분류/레시피 도출 로직
- `/home/up/ros2_ws/src/bartender-robot/src/order_integration/voice_order_test_worker.py`
  - 별도 프로세스 워커 (stdin JSON 입력, stdout JSON 로그 출력)
- `/home/up/ros2_ws/src/bartender-robot/src/frontend/developer_frontend.py`
  - 좌측 테스트 패널 UI 및 워커 연동

## 동작 방식

1. UI에서 `테스트 시작` 클릭
2. 프론트엔드가 `voice_order_test_worker.py`를 subprocess로 실행
3. 워커가 단계 로그를 JSON 라인으로 출력
4. UI가 로그를 실시간 표시하고 최종 메뉴/레시피를 결과창에 반영

## 현재 제한

- 마이크 실시간 수집(STT)은 비활성화
- HTML 주문 UI는 화면에서 비활성화로 표시만 함
- 로봇 명령 실행은 강제 비활성화

