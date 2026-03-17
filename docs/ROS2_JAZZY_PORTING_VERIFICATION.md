# ROS2 Jazzy 포팅 후 검증 정리

- 작성일시: 2026-03-10T14:01:48+09:00
- 대상 저장소: `bartender-robot`
- 기준 브랜치: `bartender-robot`
- 기준 커밋(변경 전 HEAD): `04d6083`
- 이번 커밋 주제: **ROS2 Jazzy 로 포팅후 검증완료**

## 1) 실행 환경 버전 총정리

### OS / 커널
- OS: Ubuntu 24.04.4 LTS (Noble Numbat)
- Kernel: `6.17.0-14-generic`
- Arch: `x86_64`

### ROS2
- ROS_DISTRO: `jazzy`
- ROS_VERSION: `2`
- ROS_PYTHON_VERSION: `3`
- `ros2 pkg list` 개수: `459`

### ROS 주요 패키지(설치 버전)
- `ros-jazzy-ros2cli`: `0.32.8-1noble.20260126.192739`
- `ros-jazzy-rclpy`: `7.1.9-1noble.20260126.174822`
- `ros-jazzy-ros-base`: `0.11.0-1noble.20260126.203129`
- `ros-jazzy-desktop`: `0.11.0-1noble.20260126.203157`
- `ros-jazzy-librealsense2`: `2.56.4-1noble.20260121.175802`
- `ros-jazzy-realsense2-camera`: `4.56.4-1noble.20260126.181858`
- `ros-jazzy-realsense2-description`: `4.56.4-1noble.20260126.192347`

### Python 런타임/라이브러리
- Python: `3.12.3`
- pip: `25.3`
- ultralytics: `8.3.228`
- torch: `2.9.1+cu130`
- opencv-python: `4.12.0.88`
- numpy: `1.26.4`
- PyQt5: `5.15.10`

### 연동 저장소/패키지 버전
- `/home/up/ros2_ws/src/doosan-robot2`
  - branch: `jazzy`
  - commit: `3ef2717`
  - `dsr_common2`: `1.1.0`
  - `dsr_controller2`: `2.33.0`
  - `dsr_bringup2`: `1.1.0`
- `/home/up/ros2_ws/src/od-realsense`
  - detached HEAD: `8574b22`

## 2) 변경사항 요약 (폴더 전체 관점)

### 런치/부트스트랩
- `run_bartender.py`
  - ROS 환경 미적용 상태에서도 자동 재실행(`source /opt/ros/jazzy`, workspace setup)하도록 부트스트랩 추가.
- `launch/system_launch.py`
  - real/virtual 모드별 `robot_gz` 기본값 정리(real=false, virtual=true).
  - `rt_host` 인자 별칭 및 로컬 NIC 자동 탐지 추가.
  - 프론트엔드 종료 시 Gazebo 관련 잔여 프로세스 정리 강화(`gz sim/gui`, `ign gazebo`, `gzserver/client`).

### 백엔드 서비스/로봇 연결
- `src/backend/task_backend_node.py`
  - DSR 서비스 호출을 `dsr_controller2/*` 경로 기준으로 통합.
  - 서비스 클라이언트 캐시를 단일 객체에서 다중 후보 캐시로 재구성.
  - 모드/리셋/TCP 조회/설정 서비스 응답 처리 안정화.
- `src/backend/gripper_drl_controller.py`
  - 그리퍼 DRL 서비스 경로를 `/<robot_id>/dsr_controller2/drl/drl_start`로 고정.
  - 무한 대기 제거: `GRIPPER_DRL_WAIT_TIMEOUT_SEC` 기반 타임아웃 추가(기본 25초).

### 프론트엔드 안정성/비전
- `src/frontend/developer_frontend.py`
  - 타이머 콜백 공통 보호 래퍼 `_safe_ui_tick()` 추가.
  - 종료 레이스(`RCLError`, context invalid) 예외를 UI 크래시로 전파하지 않도록 처리.
  - 비전 depth 토픽을 실시간 생존 토픽 우선으로 선택하도록 개선.
  - 비전 메타 프로세스 즉시 종료 시 로그로 원인 추적 가능하게 개선.

### 비전 노드
- `src/vision/glass_fill_level.py`
- `src/vision/glass_fill_level_preview.py`
  - 구버전 모델 직렬화 별칭(`Segment26`, `Proto26`) 호환 패치 추가.
- `src/vision/vision_meta_common.py`
  - depth 지연/미수신 시에도 color 기반 메타 파이프라인이 멈추지 않도록 변경.

### 설정 파일
- `config/parameter.csv`
  - 홈 조인트 값 및 상단 상태 토글 값이 현장 운용값으로 갱신됨.
  - 주의: 장비별 값이므로 다른 장비에 동일 적용 시 재검증 필요.

### 개발환경 설정
- `.vscode/settings.json`
  - VSCode ROS 배포판을 `jazzy`로 명시.

## 3) 문서/패치 아티팩트

- 포팅 검증 문서: `docs/ROS2_JAZZY_PORTING_VERIFICATION.md`
- 패치 번들: `docs/patches/ros2_jazzy_porting_20260310.patch`
  - lines: `928`
  - bytes: `40223`
  - sha256: `df7b8799b35d53155a02cfe1ed3c257f8e73ad3cd516214471c2ea1b01ed8b08`

## 4) 검증 결과

### 정적 검증
- 다음 파일들 `py_compile` 통과:
  - `run_bartender.py`
  - `launch/system_launch.py`
  - `src/backend/gripper_drl_controller.py`
  - `src/backend/task_backend_node.py`
  - `src/frontend/developer_frontend.py`
  - `src/vision/glass_fill_level.py`
  - `src/vision/glass_fill_level_preview.py`
  - `src/vision/vision_meta_common.py`

### 런타임 포인트 검증
- DSR DRL 서비스 확인:
  - `/dsr01/dsr_controller2/drl/drl_start` 경로가 실제 서비스로 동작.
  - 기존 `/dsr01/drl/drl_start` 경로는 런타임에 미사용/미가용 케이스 확인.
- 비전 ML 표시는 `/vision2/volume/meta` 수신 여부가 핵심이며, 미수신 시 UI 상태 패널/로그로 추적 가능.

## 5) 운영 시 주의사항

1. `config/parameter.csv`는 장비 캘리브레이션/홈위치/토글 상태를 포함하므로 장비별 백업 후 적용.
2. real 모드 기본이 `robot_gz=false`로 변경되어, 시뮬 동시구동 필요 시 실행 인자에서 `robot_gz:=true` 명시.
3. 그리퍼 DRL 서비스 타임아웃(기본 25초) 초과 시 초기화 실패로 반환되므로, 네트워크/컨트롤러 상태 먼저 점검.
4. 비전2 용량(`volume_ml`)은 `bottle`+`liquid(soju/beer)` 동시 감지 조건을 만족해야 계산됨.
5. VSCode 실행 시에도 ROS 환경 자동 부트스트랩되지만, 별도 venv 사용 시 `rclpy/sensor_msgs` 가시성 충돌 여부 확인 필요.
