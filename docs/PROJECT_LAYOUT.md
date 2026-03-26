# Project Layout

Last updated: 2026-03-26

## English

### Top-level

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

### Runtime-critical paths

- `run_bartender.py`
- `launch/system_launch.py`
- `src/frontend/developer_frontend.py`
- `src/frontend/user_frontend.py`
- `src/backend/task_backend_node.py`
- `src/bartender_action/*`
- `src/order_integration/*`
- `src/vision/*`

### Directory roles

- `assets/models/`: vision model files (`cam_1.pt`, `cam_2.pt`, etc.)
- `config/`: runtime parameters, calibration files, motion/offset configs
- `launch/`: ROS2 launch entrypoints
- `scripts/`: utility scripts (vendor patch apply)
- `vendor/doosan-robot2/*.patch`: external vendor patch files

## Korean (한국어)

### 최상위 구조

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

### 실행 핵심 경로

- `run_bartender.py`
- `launch/system_launch.py`
- `src/frontend/developer_frontend.py`
- `src/frontend/user_frontend.py`
- `src/backend/task_backend_node.py`
- `src/bartender_action/*`
- `src/order_integration/*`
- `src/vision/*`

### 디렉터리 역할

- `assets/models/`: 비전 모델 파일(`cam_1.pt`, `cam_2.pt` 등)
- `config/`: 런타임 파라미터, 캘리브레이션, 모션/오프셋 설정
- `launch/`: ROS2 런치 엔트리포인트
- `scripts/`: 유틸 스크립트(벤더 패치 적용)
- `vendor/doosan-robot2/*.patch`: 외부 벤더 패치 파일
