# Motion Sequence Edit/Test Guide

Last updated: 2026-03-26

## English

### Scope

This guide covers where to edit robot motion definitions and how to test safely with current code.

### Main edit points

- Motion planning:
  - `src/bartender_action/robot_action_planner.py`
  - key areas: start / ingredient (pick-pour-return) / finish blocks
- UI mapping:
  - `src/frontend/developer_frontend.py`
  - robot action pose table rows and editing hooks
- Execution gate:
  - `src/backend/task_backend_node.py`
  - sequence entry, safety checks, gripper readiness

### Current behavior notes

- Backend sequence API remains the control source of truth.
- `robot_action_only=true` path is available for robot-only sequence checks.
- `execute_robot_action=false` can be used for planning-only validation.
- Finish-stage cup pick/delivery behavior may be disabled depending on planner block status; validate in logs before field test.

### Recommended test order

1. Syntax check

```bash
python3 -m py_compile src/frontend/developer_frontend.py
python3 -m py_compile src/backend/task_backend_node.py
python3 -m py_compile src/bartender_action/robot_action_planner.py
```

2. Plan-only test (`execute_robot_action=false`)
3. Real motion test for single ingredient
4. Multi-ingredient test
5. Optional finish-stage validation if enabled

### Artifacts

- Pose config: `config/robot_action_pose_config.json`
- Menu offset config: `config/menu_xyz_offsets.json`
- UI screenshots: `docs/img/*`

## Korean (한국어)

### 범위

이 문서는 현재 코드 기준으로 모션 정의를 어디서 수정하고, 어떻게 안전하게 검증할지 정리합니다.

### 주요 수정 지점

- 모션 플래너:
  - `src/bartender_action/robot_action_planner.py`
  - 시작/재료별(PICK-POUR-RETURN)/종료 블록
- UI 매핑:
  - `src/frontend/developer_frontend.py`
  - 액션 포즈 표 행 정의 및 편집 연결
- 실행 게이트:
  - `src/backend/task_backend_node.py`
  - 실행 진입, 안전 체크, 그리퍼 준비 확인

### 현재 동작 메모

- 백엔드 시퀀스 API가 단일 제어 기준입니다.
- `robot_action_only=true` 경로로 로봇 동작만 점검할 수 있습니다.
- `execute_robot_action=false`로 계획 검증 전용 실행이 가능합니다.
- 종료 단계 컵 전달은 플래너 블록 상태에 따라 비활성일 수 있으므로, 실기 전에 로그로 확인하세요.

### 권장 테스트 순서

1. 문법 점검

```bash
python3 -m py_compile src/frontend/developer_frontend.py
python3 -m py_compile src/backend/task_backend_node.py
python3 -m py_compile src/bartender_action/robot_action_planner.py
```

2. 계획 검증(`execute_robot_action=false`)
3. 단일 재료 실동작 검증
4. 다중 재료 검증
5. 필요 시 종료 단계(컵 전달) 검증

### 관련 산출물

- 포즈 설정: `config/robot_action_pose_config.json`
- 메뉴 보정값: `config/menu_xyz_offsets.json`
- UI 스크린샷: `docs/img/*`
