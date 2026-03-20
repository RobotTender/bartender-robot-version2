# 바텐더 로봇 프로젝트

두산 로봇, RealSense 2대, 개발자 UI, 사용자 Web UI, 음성주문(STT->LLM->레시피), 비전 메타 파이프라인을 통합 실행하는 프로젝트입니다.

## 핵심 구성

- 로봇 제어 백엔드
  - 로봇 상태/모드/좌표 수집, 이동 명령, 그리퍼 초기화/제어를 담당합니다.
- 개발자 UI
  - 로봇/비전/음성주문 상태 확인, 테스트 실행, 로그 디버깅을 담당합니다.
- 사용자 Web UI
  - 최종 사용자 주문 진입 화면을 제공합니다.
  - 현재는 `USER_FRONTEND_ENABLED`(하위호환: `VOICE_ORDER_WEBUI_ENABLED`) 플래그로 진입 허용/차단을 제어합니다.
- 음성주문 통합
  - 백엔드 요청 -> 음성처리 워커 -> Gemini STT -> 주문 분류(LLM 보조) -> 레시피 결과를 반환합니다.
- 바텐더 시퀀스 매니저(백엔드)
  - `BartenderSequenceManager`가 오토/메뉴얼 공용 시퀀스를 실행합니다.
  - 안전 판정/정지 권한은 백엔드만 보유하고, 프론트엔드는 시작/중지 요청과 상태 표시만 수행합니다.
- 비전 메타 파이프라인
  - 비전1 객체 메타, 비전2 용량 메타를 발행하고 UI에서 오버레이합니다.
- 캘리브레이션
  - Eye-to-Hand 행렬 계산/저장/적용으로 비전 좌표를 로봇 좌표로 변환합니다.

## 실행

### 1) 환경 로드

```bash
source /opt/ros/humble/setup.bash
source <ros2_ws>/install/setup.bash
```

### 2) 기본 실행

```bash
cd <project-root>
python3 run_bartender.py
```

기본 launch 인자:

```bash
run_robot:=true
robot_mode:=real
robot_host:=110.120.1.68
robot_model:=e0509
run_sensors:=true
run_frontend:=true
run_user_frontend:=true
webui_host:=0.0.0.0
webui_port:=8000
sequence_api_host:=127.0.0.1
sequence_api_port:=8765
```

### 3) 자주 쓰는 실행 예시

실제 로봇:

```bash
python3 run_bartender.py robot_mode:=real robot_host:=110.120.1.68 robot_model:=e0509
```

가상 로봇:

```bash
python3 run_bartender.py robot_mode:=virtual robot_model:=e0509
```

로봇 bringup 제외:

```bash
python3 run_bartender.py run_robot:=false
```

사용자 Web UI 제외:

```bash
python3 run_bartender.py run_user_frontend:=false
```

## 주문 기능 환경변수(.env)

`.env` 파일은 프로젝트 루트(`<project-root>/.env`)에 두고 사용합니다.

```bash
cp .env.example .env
```

- `GOOGLE_API_KEY` 또는 `GEMINI_API_KEY`
- `OPENAI_API_KEY`
- `VOICE_ORDER_MODEL`
- `VOICE_ORDER_STT_MODEL`
- `VOICE_ORDER_STT_RETRIES`
- `USER_FRONTEND_ENABLED` (또는 `VOICE_ORDER_WEBUI_ENABLED`)
- `USER_FRONTEND_HOST` (또는 `VOICE_ORDER_WEBUI_HOST`)
- `USER_FRONTEND_PORT` (또는 `VOICE_ORDER_WEBUI_PORT`)
- `BARTENDER_SEQUENCE_API_HOST`
- `BARTENDER_SEQUENCE_API_PORT`
- `VOICE_ORDER_CYCLE_INTERVAL_MS`

`.env.example`은 키 값을 공백으로 둔 템플릿이며, 실제 키는 `.env`에만 입력합니다.

## 프로그램 구조(요약)

```text
run_bartender.py
  -> launch/system_launch.py
     -> doosan bringup
     -> realsense launch
     -> developer_frontend.py
     -> user_frontend.py (최종 사용자 Web UI)

developer_frontend.py
  -> backend(task_backend_node.py).run_voice_order_runtime(...)
     -> subprocess: voice_order_worker.py
        -> gemini_stt_pipeline.py
        -> voice_order_pipeline.py
        -> recipe result
  -> backend(task_backend_node.py).BartenderSequenceManager
     -> 시퀀스 상태/로그/안전정지 권한 단일화

백엔드 시퀀스 제어 API:
- `GET /api/sequence/state`
- `POST /api/sequence/start`
- `POST /api/sequence/stop`
```

상세 구조/상태:

- 구조와 폴더 역할: [docs/PROJECT_LAYOUT.md](docs/PROJECT_LAYOUT.md)
- 런타임/프로세스 구조: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- 주문기능 머지 1차 상태: [docs/ORDER_FEATURE_MERGE_PHASE1.md](docs/ORDER_FEATURE_MERGE_PHASE1.md)
- 배포 절차: [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)
- 벤더 패치: [docs/VENDOR_PATCHES.md](docs/VENDOR_PATCHES.md)
- 모션 시퀀스 수정/테스트 상세 가이드: [docs/MOTION_SEQUENCE_EDIT_TEST_GUIDE.md](docs/MOTION_SEQUENCE_EDIT_TEST_GUIDE.md)

## 모션 시퀀스 최신 업데이트 (2026-03-20)

- 액션 포지션 설정창 탭 구조 정리
  - `소주/맥주/글라스잔` 탭 분리, 타입 축약 표기(`posj`, `posx`, `g_open`, `g_close` 등)
- 시퀀스 표기 정리
  - 행 전체 선택/하이라이트 및 같은 탭 내 동일 변수 참조행 동시 하이라이트
  - `(return_ref)` 표기 제거(변수명 표시 기준)
- 참조형 표준화
  - `service_ready_posj`는 상단 기준값 1개(수정/티칭 가능) + 중간 참조행(이동/실행 참조) 구조로 정리
  - return 역순 참조행, 그리퍼 오픈/파지 행은 참조형으로 통일
  - 참조형은 `현재값 '-'`, `설명 '위치참조'`, `상태 '참조'`, 이동/실행만 허용
- 시퀀스 항목 보강
  - 글라스 DELIVERY에 `전달 위치 이탈` 항목 추가
  - PICK/RETURN에서 그리퍼 순서 표기를 실제 흐름에 맞게 정리
- 현재 실행 기준 주의
  - `robot_action_planner.py`의 `_append_finish_sequence(...)` 내부 CUP PICK/DELIVERY 모션 블록은 주석 처리 상태
  - 기본 planner 실행에서는 `[glass] 완성컵 처리 시퀀스 시작` 로그만 남고 컵 전달 step은 실행되지 않음
- 실행 안정성 보강
  - 시퀀스 실행 직전 그리퍼 초기화 확인 및 필요 시 재초기화 후 진행
  - 실시간 pour 반응성 파라미터(`BARTENDER_LIVE_POUR_*`) 기본값 정리
- 유지된 동작(미변경)
  - 백엔드 실행 진입점(`run_bartender_first_ingredient_action`)과 시퀀스 API(`/api/sequence/start|stop|state`) 구조 유지
- UI 정리/제거 항목
  - 액션 포지션 표의 `수정` 버튼 제거(값 셀 클릭 방식으로 통일)
  - `값클릭수정`, `이동전용`, `보기` 텍스트 라벨 제거
  - 그리퍼 오픈/파지 결합 행(`gripper_actions`) 표기 제거, 단일 액션 행으로 분리

위 항목의 실제 수정/검증 절차(스크린샷 삽입 위치 포함)는 아래 문서를 참고하세요.

- [docs/MOTION_SEQUENCE_EDIT_TEST_GUIDE.md](docs/MOTION_SEQUENCE_EDIT_TEST_GUIDE.md)

## 현재 주의사항

- Doosan 동작에는 vendor patch 적용이 필요합니다.
- Web UI는 기본값이 비활성화(`USER_FRONTEND_ENABLED=0`, 하위호환 `VOICE_ORDER_WEBUI_ENABLED=0`)라서 접근해도 주문은 차단될 수 있습니다.
- 사용자 Web UI의 `/api/control/start`는 현재 백엔드 로봇 실행과 직접 연결되지 않은 상태입니다.
- 음성주문 결과는 현재 백엔드 메모리 스냅샷과 UI 이벤트 기반이며, ROS 토픽/서비스 표준 인터페이스는 아직 확정 전입니다.

## 리모트 최신 커밋 대비 변경 요약 (2026-03-17)

비교 기준: `origin/dev` 최신 커밋 `89cb9e6` 대비 로컬 작업트리.

- 아키텍처/실행 흐름 확장
  - `task_backend_node.py`, `developer_frontend.py` 중심으로 바텐더 시퀀스 API 연동과 상태 표현 로직이 크게 확장됨.
  - 사용자 Web UI 진입점(`src/frontend/user_frontend.py`)과 주문 라우팅/워커 분리 파일이 추가됨.
- 주문 기능 파이프라인 정리
  - STT/Gemini, LLM 분류, TTS 관련 모듈(`gemini_stt_pipeline.py`, `voice_order_worker.py`, `openai_tts.py`, `voice_order_route.py`)이 추가/재구성됨.
  - 기존 `voice_order_test_worker.py`는 제거되고 실제 워커 경로로 통합됨.
- 비전/캘리브레이션 데이터 갱신
  - 비전/캘리브레이션 관련 코드와 파라미터가 업데이트됨.
  - 구 캘리브레이션 매트릭스 일부는 정리되고 신규 매트릭스 파일이 추가됨.
- 문서/실행 가이드 업데이트
  - `README.md`, `docs/*` 다수 문서에서 구조도, 배포/테스트 절차, 운영 메모를 최신 흐름에 맞춰 정리함.

## 남은 작업

- 모션 테스트(Motion Test) 미완료: 실제 로봇 기준 동작 시퀀스 최종 검증이 아직 남아 있음.
