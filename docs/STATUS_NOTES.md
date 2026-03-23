# 현재 상태 노트

기준일: 2026-03-23

## 릴리즈 정리 반영 사항

- 프론트엔드 구조 단일화
  - `assets/frontend/developer_frontend.ui` 제거
  - 개발자 UI 구조는 `src/frontend/developer_frontend_ui_runtime.py` + 동적 로직(`src/frontend/developer_frontend.py`)로 일원화
- 비전 preview 유틸 정리
  - `src/vision/drink_detection_preview.py`
  - `src/vision/glass_fill_level_preview.py`
  - 위 2개 파일 삭제(실행 경로/런치 미사용)
- 미사용 프론트 코드 제거
  - `developer_frontend.py`의 미사용 `preview_point` 콜백 및 관련 필드 정리
- 레이아웃 보정(최근 UI 이슈 대응)
  - 해상도 비율 기반으로 로봇 패널 표/컨트롤 배치 계산 보정
  - 표 행 높이 계산에서 하단 빈공간이 남는 케이스 제거

## 문서 정리 반영 사항

- 현재 구조 기준으로 문서 정합성 갱신
  - `README.md`
  - `docs/PROJECT_LAYOUT.md`
  - `docs/ARCHITECTURE.md`
  - `docs/ORDER_FEATURE_MERGE_PHASE1.md`
  - `docs/MOTION_SEQUENCE_EDIT_TEST_GUIDE.md`
  - `docs/ROS2_JAZZY_PORTING_VERIFICATION.md`
- 삭제된 파일(.ui / preview) 참조 제거
- 운영에 직접 필요 없는 과거 작업트리/푸시 이력성 본문 제거
- 가이드 문구 정리
  - UI에 없는 드라이런 버튼 표현 제거
  - 계획 검증은 API 옵션(`execute_robot_action=false`) 기반으로 명시

## 현재 릴리즈 관점 체크포인트

- Doosan vendor patch 적용 여부
- `.env` API 키/호스트/포트 값 검증
- `config/parameter.csv`, `config/menu_xyz_offsets.json`, `config/robot_action_pose_config.json` 운영값 백업
- 실기 기준 모션 최종 검증(PICK/POUR/RETURN 및 예외복구)
