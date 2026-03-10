# Deployment Guide

## 1. 배포 전 전제조건

필수 설치/준비:

- ROS2 Humble
- `doosan-robot2` 소스
- `realsense2_camera` 패키지
- 이 저장소: `bartender-robot`
- 필요한 경우 모델 파일

워크스페이스 예시:

```text
<ros2_ws>/src/
  doosan-robot2/
  bartender-robot/
```

## 2. Doosan vendor patch 적용

이 저장소는 Doosan 원본만으로는 바로 동작하지 않습니다.

핵심 이유:

- `RobotState`, `RobotStateRt` 토픽이 필요함
- Gazebo/bringup 쪽 보강 patch가 필요할 수 있음

문서:

- [docs/VENDOR_PATCHES.md](VENDOR_PATCHES.md)

적용:

```bash
cd <repo-root>
./scripts/apply_doosan_vendor_patches.sh
```

## 3. 빌드

```bash
cd <ros2_ws>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source <ros2_ws>/install/setup.bash
```

## 4. 런타임 설정 확인

배포 전에 최소한 아래를 확인해야 합니다.

### 카메라 시리얼

- [config/parameter.csv](../config/parameter.csv)

확인 항목:

- `vision1_serial`
- `vision2_serial`

### 캘리브레이션 파일

- [config/calibration](../config/calibration)

확인 항목:

- 실제 사용할 행렬 파일 존재 여부
- `parameter.csv`의 활성 경로와 일치 여부

### 모델 파일

- [assets/models](../assets/models)

현재는:

- [assets/models/best.pt](../assets/models/best.pt)

## 5. 실행 방법

### 기본 실행

```bash
cd <repo-root>
python3 run_bartender.py
```

### 실제 로봇

```bash
python3 run_bartender.py robot_mode:=real robot_host:=110.120.1.68 robot_model:=e0509
```

### 가상 로봇

```bash
python3 run_bartender.py robot_mode:=virtual robot_model:=e0509
```

Gazebo 부하를 빼려면:

```bash
python3 run_bartender.py robot_mode:=virtual robot_model:=e0509 robot_gz:=false
```

### 로봇 없이 앱만 확인

```bash
python3 run_bartender.py run_robot:=false
```

## 6. 개별 기능 실행

### RealSense만

```bash
python3 launch/realsense_launch.py
```

### 객체 인식 프로세스

```bash
python3 launch/object_detection_launch.py run_drink_detection:=true
python3 launch/object_detection_launch.py run_glass_fill_level:=true
python3 launch/object_detection_launch.py run_drink_detection:=true run_glass_fill_level:=true
```

### 캘리브레이션 프로세스

```bash
python3 launch/calibration_launch.py run_vision1_calibration:=true
python3 launch/calibration_launch.py run_vision2_calibration:=true
```

## 7. 배포 체크리스트

배포 전에 아래를 확인하는 게 좋습니다.

- Doosan patch 적용 여부
- `colcon build` 성공 여부
- `config/parameter.csv` 시리얼 값 확인
- 캘리브레이션 파일 존재 여부
- 실제 모델 파일 위치 확인
- 실제 로봇 IP 확인
- USB 연결 상태 확인
  - RealSense가 USB 2.1로 붙으면 성능 저하 가능

## 8. 배포물에 포함할 것

최소 포함 권장:

- 이 저장소 전체
- Doosan patch 파일
- 배포 문서
- 실제 사용할 calibration 파일
- 실제 사용할 모델 파일

배포 문서로 같이 전달할 것:

- 실제 로봇 IP
- 로봇 모델명
- RealSense serial mapping
- 필요한 vendor branch/commit

## 9. 배포 후 첫 점검

### 1) 로봇

- `run_bartender.py`에서 bringup 정상 여부
- 상태 토픽 정상 여부

### 2) 카메라

- 비전1/비전2 raw 화면 정상 여부
- serial mapping이 맞는지

### 3) 캘리브레이션

- 캘 모드 ON/OFF 정상 여부
- 메타 수신/오버레이 정상 여부

### 4) 앱 종료

- 프론트엔드 종료 시 관련 프로세스가 같이 정리되는지
