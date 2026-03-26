# Deployment Guide

Last updated: 2026-03-26

## English

### Prerequisites

- ROS2 workspace with:
  - this repository
  - `doosan-robot2`
  - `realsense2_camera`
- Python dependencies for voice pipeline
- Correct `.env` keys for external APIs

### 1) Apply vendor patches

```bash
cd <repo-root>
./scripts/apply_doosan_vendor_patches.sh [<path-to-doosan-robot2>]
```

Default target path is `${HOME}/ros2_ws/src/doosan-robot2`.

### 2) Build workspace

```bash
cd <ros2_ws>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If Jazzy is unavailable, use your installed ROS distro.

### 3) Runtime checklist

- `config/parameter.csv`
  - `vision1_serial`, `vision2_serial`
- `config/calibration/*.txt`
  - active calibration matrix exists
- `assets/models/`
  - required model files exist (`cam_1.pt`, `cam_2.pt`)
- `.env`
  - API keys and web UI flags configured

### 4) Start

```bash
cd <repo-root>
python3 run_bartender.py
```

Variants:

```bash
python3 run_bartender.py robot_mode:=real robot_host:=<ROBOT_IP> robot_model:=e0509
python3 run_bartender.py robot_mode:=virtual robot_model:=e0509
python3 run_bartender.py run_robot:=false
```

### 5) Post-deploy validation

- Robot state and motion stop path are responsive.
- Camera streams and metadata arrive from both cameras.
- Sequence API responds (`/api/sequence/state`).
- Developer UI and user UI are reachable.

## Korean (한국어)

### 전제조건

- ROS2 워크스페이스에 아래가 포함되어야 합니다.
  - 본 저장소
  - `doosan-robot2`
  - `realsense2_camera`
- 음성 파이프라인용 Python 의존성
- 외부 API용 `.env` 키 설정

### 1) 벤더 패치 적용

```bash
cd <repo-root>
./scripts/apply_doosan_vendor_patches.sh [<doosan-robot2-경로>]
```

기본 대상 경로는 `${HOME}/ros2_ws/src/doosan-robot2`입니다.

### 2) 워크스페이스 빌드

```bash
cd <ros2_ws>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Jazzy가 없으면 설치된 ROS 배포판으로 대체하세요.

### 3) 런타임 체크리스트

- `config/parameter.csv`
  - `vision1_serial`, `vision2_serial`
- `config/calibration/*.txt`
  - 활성 캘리브레이션 행렬 파일 존재
- `assets/models/`
  - 필요한 모델(`cam_1.pt`, `cam_2.pt`) 존재
- `.env`
  - API 키 및 웹 UI 플래그 설정

### 4) 실행

```bash
cd <repo-root>
python3 run_bartender.py
```

변형 실행:

```bash
python3 run_bartender.py robot_mode:=real robot_host:=<ROBOT_IP> robot_model:=e0509
python3 run_bartender.py robot_mode:=virtual robot_model:=e0509
python3 run_bartender.py run_robot:=false
```

### 5) 배포 후 점검

- 로봇 상태 수집/모션 정지 경로 정상 응답
- 양쪽 카메라 스트림/메타 수신 정상
- 시퀀스 API 응답 확인(`/api/sequence/state`)
- 개발자 UI/사용자 UI 접근 가능
