# Doosan Vendor Patches

Last updated: 2026-03-26

## English

This project depends on patching external `doosan-robot2` sources.

### Required patches

1. `vendor/doosan-robot2/0001-dsr-controller2-state-topics.patch`
2. `vendor/doosan-robot2/0002-gazebo-startup-and-update-rate.patch`

### Apply

```bash
./scripts/apply_doosan_vendor_patches.sh [<path-to-doosan-robot2>]
```

Then rebuild your ROS2 workspace.

### Why patch externally

These changes belong to vendor packages (`dsr_controller2`, `dsr_bringup2`, etc.), not this app repository.

## Korean (한국어)

이 프로젝트는 외부 `doosan-robot2` 소스 patch 적용을 전제로 동작합니다.

### 필수 patch

1. `vendor/doosan-robot2/0001-dsr-controller2-state-topics.patch`
2. `vendor/doosan-robot2/0002-gazebo-startup-and-update-rate.patch`

### 적용 방법

```bash
./scripts/apply_doosan_vendor_patches.sh [<doosan-robot2-경로>]
```

적용 후 ROS2 워크스페이스를 다시 빌드하세요.

### 외부 patch가 필요한 이유

이 변경은 `dsr_controller2`, `dsr_bringup2` 등 벤더 패키지 자체 수정이며, 앱 저장소 코드로 대체할 수 없습니다.
