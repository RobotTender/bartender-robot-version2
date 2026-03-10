# Runtime Structure

## 실행 진입점

- 사용자 실행 파일: `run_bartender.py`
- 내부 launch entrypoint: `launch/system_launch.py`
- 로봇 bringup 계층: `dsr_bringup2/dsr_bringup2_gazebo.launch.py`
- 센서 입력 계층: `launch/realsense_launch.py`
- 객체 인식 프로세스 계층: `launch/object_detection_launch.py`
- 캘리브레이션 프로세스 계층: `launch/calibration_launch.py`

사용자 입장에서는 `run_bartender.py` 하나만 실행하면 됩니다.

기본값은 아래와 같습니다.

- `run_robot:=true`
- `robot_mode:=real`
- `robot_host:=110.120.1.68`
- `robot_model:=e0509`

## 왜 launch 파일이 여러 개인가

- `system_launch.py`
  - 전체 시스템 진입점
  - Doosan bringup, RealSense 센서, frontend를 함께 올립니다.
- `dsr_bringup2_gazebo.launch.py`
  - Doosan vendor 쪽 로봇 bringup
  - `mode:=real` 또는 `mode:=virtual`로 실제 로봇/가상 로봇을 전환합니다.
- `realsense_launch.py`
  - RealSense 2대를 직접 올리는 센서 launch입니다.
  - `run_camera1`, `run_camera2`로 개별 on/off도 가능합니다.
  - 시리얼 매핑은 `vision1_serial`, `vision2_serial`만 사용합니다.
- `object_detection_launch.py`
  - 객체 인식 프로세스 2개를 실행하는 launch입니다.
  - `run_drink_detection`, `run_glass_fill_level`로 개별 on/off가 가능합니다.
- `calibration_launch.py`
  - 캘리브레이션 프로세스 2개를 실행하는 launch입니다.
  - `run_vision1_calibration`, `run_vision2_calibration`로 개별 on/off가 가능합니다.

즉 사용자용 실행 파일은 하나이고, 내부 launch는 역할별로만 분리합니다.

## 기능 분리

- `src/frontend/`
  - 개발자용 프론트엔드
  - 화면 표시, 버튼 입력, lightweight overlay만 담당
  - 비전 화면은 RealSense raw image를 기본으로 표시함
  - calibration/object detection 메타데이터가 있으면 raw image 위에 overlay만 수행함
  - 무거운 처리는 직접 하지 않음
  - 사용하지 않는 depth/camera_info 구독과 panel render timer는 끊어서 부하를 줄임
- `src/backend/`
  - 로봇 제어, 상태 구독, pick/place, 수동 이동
  - 로봇 관련 ROS 구독/서비스는 backend에서만 관리
- `src/vision/drink_detection.py`
  - 술 객체 인식용 placeholder
  - 전용 RealSense raw 입력을 받아 meta 데이터를 publish하는 구조로 구현할 예정입니다
- `src/vision/glass_fill_level.py`
  - 잔 용량 체크용 placeholder
  - 전용 RealSense raw 입력을 받아 meta 데이터를 publish하는 구조로 구현할 예정입니다
- `src/vision/camera_eye_to_hand_robot_calibration.py`
  - RealSense + OpenCV + 로봇을 이용한 Eye-to-Hand 캘리브레이션 프로세스입니다
  - 체커보드 검출/보드 전체 데이터 계산 같은 무거운 처리는 여기서 담당합니다
- `config/`
  - 런타임 파라미터, 카메라 시리얼, 캘리브레이션 결과
- `vendor/doosan-robot2/`
  - 공유용 vendor patch

## 캘리브레이션 책임

구조 원칙은 아래처럼 가져갑니다.

- 로봇 bringup
  - Doosan 연결과 상태 토픽 제공
- 센서 입력
  - RealSense publish 전용
- 캘리브레이션
  - RealSense 입력을 받아 OpenCV 보드 검출과 변환 계산 수행
- 백엔드
  - 로봇 이동, 상태, 명령 처리
- 운영 인식
  - 이후 `drink_detection.py`, `glass_fill_level.py`가 raw image 기반으로 meta만 publish
  - frontend는 raw image 위에 detection overlay만 수행

현재 운영 인식 실행 파일은 아직 비워 둔 placeholder 상태입니다.
