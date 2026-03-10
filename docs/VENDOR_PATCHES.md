# Doosan Vendor Patches

`bartender-robot`는 Doosan 원본을 그대로 쓰지 않습니다. 아래 patch가 필요합니다.

## 필수 patch

1. `vendor/doosan-robot2/0001-dsr-controller2-state-topics.patch`
   - `RobotState`, `RobotStateRt` 토픽 발행 추가
   - `actual_tcp_position` RT key 추가
   - 현재 UI/백엔드의 상태 표시, TCP 위치 추적, 모드 판별이 여기에 의존합니다.

## 선택 patch

2. `vendor/doosan-robot2/0002-gazebo-startup-and-update-rate.patch`
   - Gazebo + emulator 시작 타이밍 안정화
   - `update_rate` 전달 보강
   - 실기만 쓰면 없어도 되지만 Gazebo를 쓰면 유지하는 편이 안전합니다.

## 적용 방법

```bash
./scripts/apply_doosan_vendor_patches.sh
```

적용 후 `colcon build`를 다시 수행해야 합니다.

## 왜 별도 vendor patch인가

이 변경은 `bartender-robot` 저장소 안으로 옮길 수 있는 앱 코드가 아니라,
`dsr_controller2`, `dsr_bringup2`, `dsr_description2` 같은 외부 vendor 패키지 자체를 수정하는 내용입니다.
