# Bartender Robot Project

Unified runtime for a Doosan robot, dual RealSense cameras, developer UI, end-user web UI, voice ordering (STT -> LLM -> recipe), and vision metadata pipeline.

## English

### Key components

- Robot backend: robot state/mode tracking, robot command handling, gripper control.
- Developer UI (`src/frontend/developer_frontend.py`): operations and debug panel.
- End-user web UI (`src/frontend/user_frontend.py`): browser ordering flow.
- Voice order worker (`src/order_integration/voice_order_worker.py`): STT + order parsing.
- Sequence manager (`src/bartender_action/bartender_sequence_manager.py`): unified start/stop/state API.
- Vision nodes (`src/vision/*.py`): object metadata, volume metadata, calibration.

### Run

```bash
python3 run_bartender.py
```

`run_bartender.py` auto-bootstraps ROS environment if needed (`/opt/ros/jazzy` first, then `/opt/ros/humble`).

Common overrides:

```bash
python3 run_bartender.py robot_mode:=real robot_host:=110.120.1.68 robot_model:=e0509
python3 run_bartender.py robot_mode:=virtual robot_model:=e0509
python3 run_bartender.py run_robot:=false
python3 run_bartender.py run_user_frontend:=false
```

Main launch args (`launch/system_launch.py`):

- `run_robot`, `robot_mode`, `robot_host`, `robot_rt_host`, `robot_model`, `robot_gz`
- `run_sensors`, `run_camera1`, `run_camera2`
- `run_frontend`, `run_user_frontend`
- `webui_host`, `webui_port`, `webui_order_start_enabled`
- `sequence_api_host`, `sequence_api_port`

Legacy aliases are still supported:

- `run_web`, `run_webui` -> `run_user_frontend`
- `run_ui` -> `run_frontend`
- `rt_host` -> `robot_rt_host`

### Environment (`.env`)

```bash
cp .env.example .env
```

Required for voice features:

- `GOOGLE_API_KEY` or `GEMINI_API_KEY`
- `OPENAI_API_KEY`

Important runtime keys:

- `VOICE_ORDER_MODEL`
- `VOICE_ORDER_STT_MODEL`
- `VOICE_ORDER_STT_RETRIES`
- `USER_FRONTEND_ENABLED` (legacy: `VOICE_ORDER_WEBUI_ENABLED`)
- `USER_FRONTEND_HOST`, `USER_FRONTEND_PORT`
- `BARTENDER_SEQUENCE_API_HOST`, `BARTENDER_SEQUENCE_API_PORT`

Note: `user_frontend.py` is launched by default, but order entry is blocked unless `USER_FRONTEND_ENABLED=1` (or legacy flag).

### Sequence API (backend)

- `GET /api/sequence/state`
- `GET /api/sequence/precheck`
- `POST /api/sequence/start`
- `POST /api/sequence/stop`
- `POST /api/sequence/reset`
- `POST /api/sequence/tts_done`

### Related docs

- `docs/PROJECT_LAYOUT.md`
- `docs/ARCHITECTURE.md`
- `docs/DEPLOYMENT.md`
- `docs/VENDOR_PATCHES.md`
- `docs/MOTION_SEQUENCE_EDIT_TEST_GUIDE.md`

## Korean (한국어)

두산 로봇, RealSense 2대, 개발자 UI, 사용자 웹 UI, 음성주문(STT -> LLM -> 레시피), 비전 메타 파이프라인을 통합 실행하는 프로젝트입니다.

### 핵심 구성

- 로봇 백엔드: 로봇 상태/모드 추적, 명령 처리, 그리퍼 제어
- 개발자 UI (`src/frontend/developer_frontend.py`): 운영/디버그 패널
- 사용자 웹 UI (`src/frontend/user_frontend.py`): 브라우저 주문 흐름
- 음성 워커 (`src/order_integration/voice_order_worker.py`): STT + 주문 파싱
- 시퀀스 매니저 (`src/bartender_action/bartender_sequence_manager.py`): 시작/중지/상태 API
- 비전 노드 (`src/vision/*.py`): 객체 메타, 용량 메타, 캘리브레이션

### 실행

```bash
python3 run_bartender.py
```

`run_bartender.py`는 ROS 환경이 누락된 경우 자동으로 부트스트랩합니다(우선순위: `/opt/ros/jazzy`, 다음 `/opt/ros/humble`).

자주 쓰는 인자:

```bash
python3 run_bartender.py robot_mode:=real robot_host:=110.120.1.68 robot_model:=e0509
python3 run_bartender.py robot_mode:=virtual robot_model:=e0509
python3 run_bartender.py run_robot:=false
python3 run_bartender.py run_user_frontend:=false
```

주요 런치 인자(`launch/system_launch.py`):

- `run_robot`, `robot_mode`, `robot_host`, `robot_rt_host`, `robot_model`, `robot_gz`
- `run_sensors`, `run_camera1`, `run_camera2`
- `run_frontend`, `run_user_frontend`
- `webui_host`, `webui_port`, `webui_order_start_enabled`
- `sequence_api_host`, `sequence_api_port`

구형 별칭도 호환됩니다:

- `run_web`, `run_webui` -> `run_user_frontend`
- `run_ui` -> `run_frontend`
- `rt_host` -> `robot_rt_host`

### 환경변수 (`.env`)

```bash
cp .env.example .env
```

음성 기능 필수 키:

- `GOOGLE_API_KEY` 또는 `GEMINI_API_KEY`
- `OPENAI_API_KEY`

주요 런타임 키:

- `VOICE_ORDER_MODEL`
- `VOICE_ORDER_STT_MODEL`
- `VOICE_ORDER_STT_RETRIES`
- `USER_FRONTEND_ENABLED` (하위호환: `VOICE_ORDER_WEBUI_ENABLED`)
- `USER_FRONTEND_HOST`, `USER_FRONTEND_PORT`
- `BARTENDER_SEQUENCE_API_HOST`, `BARTENDER_SEQUENCE_API_PORT`

참고: `user_frontend.py` 프로세스는 기본 실행되지만, `USER_FRONTEND_ENABLED=1`(또는 하위호환 키) 전까지 주문 진입은 차단됩니다.

### 백엔드 시퀀스 API

- `GET /api/sequence/state`
- `GET /api/sequence/precheck`
- `POST /api/sequence/start`
- `POST /api/sequence/stop`
- `POST /api/sequence/reset`
- `POST /api/sequence/tts_done`

### 연관 문서

- `docs/PROJECT_LAYOUT.md`
- `docs/ARCHITECTURE.md`
- `docs/DEPLOYMENT.md`
- `docs/VENDOR_PATCHES.md`
- `docs/MOTION_SEQUENCE_EDIT_TEST_GUIDE.md`
