# ROS2 Jazzy Porting Verification

Last updated: 2026-03-26

## English

### Summary

This repository currently includes Jazzy-oriented runtime and launch adjustments, with fallback compatibility in selected paths.

### Verified code-level points

- `run_bartender.py`
  - ROS bootstrap auto-detection (`jazzy` first, then other installed distros).
- `launch/system_launch.py`
  - mode-based defaults and alias overrides for launch arguments.
  - Gazebo leftover process cleanup on shutdown.
- `src/backend/task_backend_node.py`
  - Doosan integration guards and service/topic handling for runtime stability.

### Operational note

Always validate vendor package versions and patch compatibility in your deployment workspace before running on hardware.

## Korean (한국어)

### 요약

현재 저장소는 Jazzy 기준 런타임/런치 조정을 포함하고 있으며, 일부 경로는 하위 호환을 유지합니다.

### 코드 기준 검증 포인트

- `run_bartender.py`
  - ROS 환경 자동 부트스트랩(`jazzy` 우선, 이후 설치된 배포판 탐색)
- `launch/system_launch.py`
  - 모드별 기본값 및 인자 별칭 호환
  - 종료 시 Gazebo 잔여 프로세스 정리
- `src/backend/task_backend_node.py`
  - Doosan 연동 보호 로직 및 서비스/토픽 처리 안정화

### 운영 메모

실기 배포 전에는 워크스페이스의 벤더 패키지 버전과 patch 호환성을 반드시 확인하세요.
