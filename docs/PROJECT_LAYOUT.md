# Project Layout

기준일: 2026-03-11

## 최상위 구조

```text
<project-root>/
  assets/
  config/
  docs/
  launch/
  scripts/
  src/
  vendor/
  run_bartender.py
  README.md
  CONTRIBUTING.md
```

## 폴더별 역할

### `assets/` (런타임 자산)

- `assets/frontend/developer_frontend.ui`
  - 개발자 UI 기본 레이아웃 파일
  - 실행 시 일부 위치/크기는 `developer_frontend.py`가 동적으로 재배치
- `assets/models/cam_1.pt`
  - 비전1 객체 인식 기본 모델
- `assets/models/cam_2.pt`
  - 비전2 용량 인식 기본 모델
- `assets/models/best.pt`
  - 현재 코드 기본 경로에서 직접 사용하지 않음(정리 후보)

### `config/` (운영 설정/결과)

- `config/parameter.csv`
  - 카메라 시리얼, 캘리브레이션 활성 경로, UI/캘리브레이션 파라미터
- `config/calibration/*.txt`
  - Eye-to-Hand 변환 행렬 결과 파일

### `docs/` (운영/구조 문서)

- 구조: `PROJECT_LAYOUT.md`, `ARCHITECTURE.md`
- 운영 상태: `STATUS_NOTES.md`, `ORDER_FEATURE_MERGE_PHASE1.md`
- 배포: `DEPLOYMENT.md`
- 벤더 패치: `VENDOR_PATCHES.md`, `patches/*`
- 이력성 문서: `ROS2_JAZZY_PORTING_VERIFICATION.md`

### `launch/` (ROS2 실행 조립)

- `system_launch.py` (주 실행)
  - 두산 bringup + RealSense + 개발자 UI + 사용자 Web UI 실행
- `realsense_launch.py` (주 실행에서 포함)
  - 카메라 1/2 실행 및 시리얼 매핑
- `object_detection_launch.py` (수동/보조 실행)
  - 객체/용량 인식 프로세스 단독 실행용
- `calibration_launch.py` (수동/보조 실행)
  - 캘리브레이션 프로세스 단독 실행용

### `scripts/`

- `apply_doosan_vendor_patches.sh`
  - Doosan 소스 패치 적용 스크립트

### `src/` (실제 애플리케이션 코드)

- `src/backend/`
  - `task_backend_node.py`: 로봇 상태/명령, 음성주문 워커 호출, 바텐더 시퀀스 API 제공
  - `gripper_drl_controller.py`: 그리퍼 제어 유틸
- `src/bartender_action/`
  - `bartender_sequence_manager.py`: 오토/메뉴얼 공용 시퀀스 실행 및 안전판정/정지
  - `robot_action_planner.py`: 레시피/비전 기반 로봇 동작 시퀀스 플래너
- `src/frontend/`
  - `developer_frontend.py`: 개발자 UI 메인
  - `user_frontend.py`: 사용자 Web UI 서버(최종 사용자 UI)
- `src/order_integration/`
  - `gemini_stt_pipeline.py`: Gemini STT
  - `voice_order_pipeline.py`: 메뉴 분류/레시피 도출 + stage/result payload 조합
  - `voice_order_worker.py`: 백엔드가 실행하는 음성 워커
  - `voice_order_route.py`: 백엔드↔워커 subprocess 호출 래퍼
- `src/vision/`
  - `drink_detection.py`: 비전1 객체 메타 발행
  - `glass_fill_level.py`: 비전2 용량 메타 발행
  - `camera_eye_to_hand_robot_calibration.py`: 캘리브레이션 실행
  - `vision_meta_common.py`: 비전 메타 공통 베이스
  - `*_preview.py`: 단독 미리보기 도구(정리 후보)

### `vendor/`

- `vendor/doosan-robot2/*.patch`
  - 두산 패치 파일

## 현재 실행에 직접 쓰이는 엔트리포인트

- `run_bartender.py`
- `launch/system_launch.py`
- `src/frontend/developer_frontend.py`
- `src/frontend/user_frontend.py`
- `src/backend/task_backend_node.py`
- `src/order_integration/voice_order_worker.py`
- `src/frontend/user_frontend.py`
- `src/vision/drink_detection.py`
- `src/vision/glass_fill_level.py`
- `src/vision/camera_eye_to_hand_robot_calibration.py` (캘리브레이션 실행 시)

## 정리 후보(삭제 전 확인 필요)

아래는 "현재 기본 실행 경로 기준" 후보입니다. 즉시 삭제하지 말고 검증 후 결정합니다.

- `assets/models/best.pt`
  - 기본 모델 경로에서 참조되지 않음(`cam_1.pt`, `cam_2.pt` 사용)
- `src/vision/drink_detection_preview.py`
  - system launch/프론트엔드 자동 실행 경로에서 사용하지 않음
- `src/vision/glass_fill_level_preview.py`
  - system launch/프론트엔드 자동 실행 경로에서 사용하지 않음
- `launch/__pycache__/`, `src/__pycache__/`
  - 실행 중 생성되는 캐시 산출물
- `.vscode/c_cpp_properties.json`
  - 로컬 IDE 설정 파일(팀 공통 실행과 무관)

## 문서화 원칙

- 실행 구조 변경 시:
  - `README.md`
  - `docs/ARCHITECTURE.md`
  - `docs/ORDER_FEATURE_MERGE_PHASE1.md`
  - 이 3개를 함께 갱신합니다.
