# Project Layout

## 목적

이 문서는 `bartender-robot` 저장소의 폴더 역할을 빠르게 파악하기 위한 문서입니다.

## 최상위 구조

```text
<repo-root>/
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

### `assets/`

- UI 파일, 모델 파일 같은 런타임 자산 보관

현재 포함:

- [assets/frontend/developer_frontend.ui](../assets/frontend/developer_frontend.ui)
- [assets/models/best.pt](../assets/models/best.pt)

### `config/`

- 런타임 설정
- 카메라 시리얼
- 활성 캘리브레이션 경로
- 캘리브레이션 결과 파일

핵심 파일:

- [config/parameter.csv](../config/parameter.csv)
- [config/calibration](../config/calibration)

### `docs/`

- 구조 설명
- 배포 절차
- Git 업로드/협업 절차
- vendor patch 설명

### `launch/`

- ROS2 launch 진입점

현재 유지하는 launch:

- [launch/system_launch.py](../launch/system_launch.py)
- [launch/realsense_launch.py](../launch/realsense_launch.py)
- [launch/object_detection_launch.py](../launch/object_detection_launch.py)
- [launch/calibration_launch.py](../launch/calibration_launch.py)

### `scripts/`

- 반복 작업용 보조 스크립트

현재 포함:

- [scripts/apply_doosan_vendor_patches.sh](../scripts/apply_doosan_vendor_patches.sh)

### `src/`

- 실제 앱 코드

하위 구성:

- [src/frontend](../src/frontend)
  - 화면, 사용자 입력, 비전 표시, lightweight overlay
- [src/backend](../src/backend)
  - 로봇 상태, 서비스, 명령, gripper 제어
- [src/vision](../src/vision)
  - 캘리브레이션 프로세스, 객체 인식 placeholder

### `vendor/`

- 외부 vendor에 적용해야 하는 patch 보관

현재 포함:

- [vendor/doosan-robot2/0001-dsr-controller2-state-topics.patch](../vendor/doosan-robot2/0001-dsr-controller2-state-topics.patch)
- [vendor/doosan-robot2/0002-gazebo-startup-and-update-rate.patch](../vendor/doosan-robot2/0002-gazebo-startup-and-update-rate.patch)

## 파일별 진입점

### 사용자 실행

- [run_bartender.py](../run_bartender.py)

### 프론트엔드

- [src/frontend/developer_frontend.py](../src/frontend/developer_frontend.py)

### 백엔드

- [src/backend/task_backend_node.py](../src/backend/task_backend_node.py)

### 캘리브레이션 프로세스

- [src/vision/camera_eye_to_hand_robot_calibration.py](../src/vision/camera_eye_to_hand_robot_calibration.py)

### 객체 인식 placeholder

- [src/vision/drink_detection.py](../src/vision/drink_detection.py)
- [src/vision/glass_fill_level.py](../src/vision/glass_fill_level.py)

## 정리 원칙

- 사용자 진입점은 [run_bartender.py](../run_bartender.py) 하나로 유지
- 내부 실행 조립은 `launch/`에만 둠
- 실제 앱 코드는 `src/`에만 둠
- 설정/결과는 `config/`에 둠
- UI/모델 자산은 `assets/`에 둠
- 외부 vendor 수정사항은 `vendor/`에 둠
