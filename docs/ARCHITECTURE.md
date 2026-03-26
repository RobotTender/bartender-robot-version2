# Runtime Architecture

Last updated: 2026-03-26

## English

### Process topology

```text
run_bartender.py
  -> launch/system_launch.py
     -> (optional) Doosan bringup
     -> (optional) RealSense launch
     -> (optional) developer_frontend.py
     -> (optional) user_frontend.py
```

### Responsibility split

- `src/frontend/developer_frontend.py`
  - Ops dashboard, debug controls, sequence start/stop integration.
- `src/frontend/user_frontend.py`
  - End-user browser UI and `/api/control/*` endpoints.
  - Calls backend `/api/sequence/*` endpoints.
- `src/backend/task_backend_node.py`
  - Robot state/control and integration boundary.
- `src/bartender_action/bartender_sequence_manager.py`
  - Sequence state machine and HTTP sequence API server.
- `src/order_integration/voice_order_route.py` + `voice_order_worker.py`
  - Subprocess-based voice execution path.
- `src/vision/*.py`
  - Vision metadata and calibration pipelines.

### Sequence APIs

- `GET /api/sequence/state`
- `GET /api/sequence/precheck`
- `POST /api/sequence/start`
- `POST /api/sequence/stop`
- `POST /api/sequence/reset`
- `POST /api/sequence/tts_done`

### User frontend control APIs

- `POST /api/control/input`
- `GET /api/control/order_start_enabled`
- `POST /api/control/start`
- `POST /api/control/stop`
- `POST /api/control/clear`

### Notes

- Frontend start/stop requests do not bypass backend safety checks.
- `run_user_frontend=true` starts the process, but order entry still requires `USER_FRONTEND_ENABLED=1`.

## Korean (한국어)

### 프로세스 구조

```text
run_bartender.py
  -> launch/system_launch.py
     -> (선택) Doosan bringup
     -> (선택) RealSense launch
     -> (선택) developer_frontend.py
     -> (선택) user_frontend.py
```

### 역할 분리

- `src/frontend/developer_frontend.py`
  - 운영 대시보드, 디버그 제어, 시퀀스 시작/중지 연동
- `src/frontend/user_frontend.py`
  - 최종 사용자 브라우저 UI, `/api/control/*` 제공
  - 내부적으로 백엔드 `/api/sequence/*` 호출
- `src/backend/task_backend_node.py`
  - 로봇 상태/제어 통합 경계
- `src/bartender_action/bartender_sequence_manager.py`
  - 시퀀스 상태머신 및 HTTP 시퀀스 API 서버
- `src/order_integration/voice_order_route.py` + `voice_order_worker.py`
  - subprocess 기반 음성 처리 경로
- `src/vision/*.py`
  - 비전 메타/캘리브레이션 파이프라인

### 시퀀스 API

- `GET /api/sequence/state`
- `GET /api/sequence/precheck`
- `POST /api/sequence/start`
- `POST /api/sequence/stop`
- `POST /api/sequence/reset`
- `POST /api/sequence/tts_done`

### 사용자 UI 제어 API

- `POST /api/control/input`
- `GET /api/control/order_start_enabled`
- `POST /api/control/start`
- `POST /api/control/stop`
- `POST /api/control/clear`

### 참고

- 프론트엔드 시작/중지 요청은 백엔드 안전 체크를 우회하지 않습니다.
- `run_user_frontend=true`여도 `USER_FRONTEND_ENABLED=1` 전까지 주문 진입은 차단됩니다.
