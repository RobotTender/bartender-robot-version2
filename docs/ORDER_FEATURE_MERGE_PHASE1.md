# Order Feature Merge - Phase 1 Notes

Last updated: 2026-03-26

## English

### Completed in phase 1

- Voice-order modules consolidated under `src/order_integration/`.
- User web UI entrypoint split into `src/frontend/user_frontend.py`.
- Sequence integration path standardized through backend sequence APIs.
- Developer UI includes voice-order debug and robot-action-only test entry.

### Current status

- Voice request execution: backend -> worker subprocess -> result payload.
- User UI start/stop: `/api/control/*` -> backend `/api/sequence/*` bridge.
- Safety authority: backend/sequence manager only.

### Open items

- Final ROS-standard interface for order result sharing is not fixed.
- Voice quality and operational prompts still need field tuning.
- Hardware-site defaults (`parameter.csv`, offsets, calibration) remain site-specific.

## Korean (한국어)

### 1차 완료 항목

- 음성주문 모듈을 `src/order_integration/`로 통합했습니다.
- 사용자 웹 UI 엔트리포인트를 `src/frontend/user_frontend.py`로 분리했습니다.
- 시퀀스 연동 경로를 백엔드 시퀀스 API 기반으로 표준화했습니다.
- 개발자 UI에 음성 디버그 및 로봇동작 단독 테스트 진입을 포함했습니다.

### 현재 상태

- 음성 요청 실행: 백엔드 -> 워커 subprocess -> 결과 payload
- 사용자 UI 시작/중지: `/api/control/*` -> 백엔드 `/api/sequence/*` 브리지
- 안전 권한: 백엔드/시퀀스 매니저 단일 보유

### 남은 항목

- 주문 결과 공유용 ROS 표준 인터페이스는 아직 확정 전입니다.
- 음성 품질/운영 프롬프트는 현장 튜닝이 추가로 필요합니다.
- `parameter.csv`, 오프셋, 캘리브레이션은 장비별 값으로 관리해야 합니다.
