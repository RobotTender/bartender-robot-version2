# bartender-robot

바텐더 로봇 런타임 저장소입니다. 이 저장소는 `Doosan bringup + RealSense + frontend/backend/vision` 앱 코드를 한곳에 정리한 독립 repo입니다.

## 개발 목적

- Doosan 로봇, RealSense, 프론트엔드/백엔드/비전 기능을 하나의 실행 환경으로 통합합니다.
- 작업자가 로봇 상태와 비전 상태를 한 화면에서 확인하고 직접 제어할 수 있는 UI를 제공합니다.
- 비전 좌표계를 로봇 좌표계로 변환해 클릭 이동과 캘리브레이션 작업을 실제 운용 흐름에 맞게 구현합니다.
- 로봇 연결, 시작 순서, 종료 처리까지 포함해 현장 사용 안정성을 강화합니다.

## 주요 기능

- 로봇 제어 UI
  - 로봇 연결상태, 로봇상태, 로봇모드, 제어모드, 현재 TCP, 조인트값, 좌표값을 실시간으로 표시합니다.
  - 홈 이동, 조인트 이동, 좌표계 이동, 속도 조절, 수동/오토 모드 전환을 지원합니다.
- 비전 UI
  - 비전1/비전2 패널을 분리하고, RealSense raw 화면 위에 메타데이터 오버레이를 표시합니다.
  - 비전1은 객체인식, 비전2는 용량인식 역할로 구분해 동작합니다.
  - 클릭 좌표의 비전 XYZ 확인과 로봇 이동 연계를 지원합니다.
  - TF 모드와 런타임 비전 모드를 구분해 필요한 UI만 표시합니다.
- 비전-로봇 좌표변환
  - Eye-to-Hand 캘리브레이션 기반으로 `비전 좌표계 -> 로봇 좌표계` 변환 행렬을 계산/저장/적용합니다.
  - 활성 캘리브레이션 파일을 관리하고, 클릭한 비전 좌표를 로봇 좌표로 변환해 이동에 활용합니다.
- 안정화 및 운영 기능
  - RobotState watchdog으로 연결 끊김 감지 및 재확인 로직을 보강했습니다.
  - 런치 종료 시 Gazebo 및 관련 잔여 프로세스를 정리합니다.
  - 운영 메모와 검증 상태를 문서로 관리합니다.

## 빠른 실행

### 환경 로드

```bash
source /opt/ros/humble/setup.bash
source <ros2_ws>/install/setup.bash
```

### 기본 실행

```bash
cd <repo-root>
python3 run_bartender.py
```

기본값:

```bash
run_robot:=true
robot_mode:=real
robot_host:=110.120.1.68
robot_model:=e0509
run_sensors:=true
run_frontend:=true
```

### 자주 쓰는 실행 예시

실제 로봇:

```bash
python3 run_bartender.py
python3 run_bartender.py robot_mode:=real robot_host:=110.120.1.68 robot_model:=e0509
```

가상 로봇:

```bash
python3 run_bartender.py robot_mode:=virtual robot_model:=e0509
python3 run_bartender.py robot_mode:=virtual robot_model:=e0509 robot_gz:=false
```

로봇 bringup 없이:

```bash
python3 run_bartender.py run_robot:=false
```

센서 없이:

```bash
python3 run_bartender.py run_sensors:=false
```

프론트엔드 없이:

```bash
python3 run_bartender.py run_frontend:=false
```

## 주요 폴더

```text
<repo-root>/
  assets/      UI 파일, 모델 파일
  config/      parameter.csv, calibration 결과
  docs/        구조/배포/Git 문서
  launch/      시스템/센서/비전 launch
  scripts/     배포 보조 스크립트
  src/         frontend/backend/vision 코드
  vendor/      Doosan vendor patch
  run_bartender.py
```

상세 설명:

- [docs/PROJECT_LAYOUT.md](docs/PROJECT_LAYOUT.md)
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)

## 실행 entrypoint

- [run_bartender.py](run_bartender.py)
  - 사용자용 단일 실행 파일
- [launch/system_launch.py](launch/system_launch.py)
  - Doosan bringup, RealSense, frontend 실행
- [launch/realsense_launch.py](launch/realsense_launch.py)
  - RealSense 2대 실행
- [launch/object_detection_launch.py](launch/object_detection_launch.py)
  - 객체 인식 프로세스 실행
- [launch/calibration_launch.py](launch/calibration_launch.py)
  - 캘리브레이션 프로세스 실행

## 실행/배포/Git 문서

- 구조 설명: [docs/PROJECT_LAYOUT.md](docs/PROJECT_LAYOUT.md)
- 런타임 구조: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- 배포 절차: [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)
- Doosan patch: [docs/VENDOR_PATCHES.md](docs/VENDOR_PATCHES.md)
- Git 업로드 절차: [docs/GIT_WORKFLOW.md](docs/GIT_WORKFLOW.md)
- 현재 검증/주의사항: [docs/STATUS_NOTES.md](docs/STATUS_NOTES.md)
- 협업 규칙: [CONTRIBUTING.md](CONTRIBUTING.md)

## 현재 주의사항

- Doosan 원본은 vendor patch 적용이 필요합니다.
- `config/parameter.csv`의 `vision1_serial`, `vision2_serial`이 실제 카메라와 맞아야 합니다.
- 비전1 객체인식은 [src/vision/drink_detection.py](src/vision/drink_detection.py), 비전2 용량인식은 [src/vision/glass_fill_level.py](src/vision/glass_fill_level.py)에서 메타를 발행하고 frontend가 이를 overlay하는 구조입니다.
- 모델 파일은 [assets/models/cam_1.pt](assets/models/cam_1.pt), [assets/models/cam_2.pt](assets/models/cam_2.pt)를 사용합니다.
- 저장소 내부 경로는 절대경로가 아니라 `__file__` 기준 상대경로로 계산합니다.
- 캘리브레이션 기능 테스트 완료 상태는 [docs/STATUS_NOTES.md](docs/STATUS_NOTES.md)에 기록합니다.
- 운영 시 목표 포지션 정확도 재검증과 모델별 offset 관리가 필요합니다.
- 현재 TF 기능은 ROS 표준 tf broadcaster 중심이라기보다, 캘리브레이션 행렬 기반 좌표변환 기능에 가깝습니다.
