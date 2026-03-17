# 주문 기능 머지 1차 정리

기준일: 2026-03-11

이 문서는 커밋 전 점검용으로 현재 "무엇을 완료했고, 무엇이 남았는지"를 정리합니다.

## 1) 1차 완료 범위

### 코드 구조 통합

- 음성주문 관련 코드를 `src/order_integration/`로 통합
- 백엔드 음성 처리 엔트리를 `task_backend_node.py`의 `run_voice_order_runtime()`로 일원화
- 사용자 Web UI 엔트리를 `src/frontend/user_frontend.py`로 분리

### 시스템 실행 경로

- `run_bartender.py` -> `launch/system_launch.py` 단일 진입 유지
- `system_launch.py`에서 사용자 Web UI 프로세스 실행 추가
- `run_web`, `run_webui` 구형 인자를 `run_user_frontend`로 정리

### 음성 처리 책임 분리

- 백엔드: 요청 전달/결과 수신 및 보관
- 워커: 마이크 입력(STT), LLM 분류, 레시피 도출
- 로봇 모션 명령은 음성 워커 경로에서 제거

### 개발자 UI 디버깅 기능

- 음성주문 연결 ON/OFF
- 마이크 입력 요청 버튼
- stage/event 로그 표시
- 주문 결과(메뉴/상태/TTS/레시피) 표시
- 업데이트 주기(ms) 표시

## 2) 현재 동작 확인 포인트

### 필수 환경변수

- `GOOGLE_API_KEY` 또는 `GEMINI_API_KEY`
- `OPENAI_API_KEY`

### 필수 패키지

- `google-genai`
- `SpeechRecognition`
- `openai`
- `python-dotenv`

### 실행 확인

1. `python3 run_bartender.py` 실행
2. 개발자 UI에서 음성주문 ON 후 "마이크 입력 시작" 테스트
3. 사용자 Web UI 주소 접속(`http://<host>:<port>`)
4. `VOICE_ORDER_WEBUI_ENABLED=0`일 때 차단 화면 노출 확인

## 3) 아직 안 된 기능(현재 기준)

- 사용자 Web UI `/api/control/start` -> 백엔드 로봇 실행 파이프라인 연동
- 음성주문 결과의 ROS 서비스/토픽 공식 인터페이스 확정
- 음성주문 결과를 실제 제조 시퀀스(모션 계획)로 연결하는 단계
- 사용자 Web UI TTS를 실제 음성 합성 엔진으로 교체(현재 tone wav)

## 4) 구조/운영 리스크

- `.ui` 파일과 런타임 코드 배치가 혼합되어 있어 디자인 변경 시 코드 검토가 필요
- 음성주문은 외부 API 키와 네트워크 상태에 의존
- 워커 subprocess 타임아웃(40s) 기준에서 장시간 STT 지연 시 실패 가능

## 5) 정리 후보(삭제 보류)

- `assets/models/best.pt`
- `src/vision/drink_detection_preview.py`
- `src/vision/glass_fill_level_preview.py`
- 로컬/캐시 산출물(`__pycache__`, 일부 `.vscode/*`)

삭제 전 반드시 확인:

- 수동 테스트 스크립트로 쓰는 사람이 없는지
- 문서/운영 스크립트에서 참조 중인지
- 배포 패키지에 포함해야 하는지

## 6) 커밋 전 체크리스트

- [ ] `README.md`, `docs/ARCHITECTURE.md`, `docs/PROJECT_LAYOUT.md` 내용 일치
- [ ] `.env` 키 이름과 코드 참조 키 일치
- [ ] 사용자 Web UI 포트/호스트 문서와 launch 기본값 일치
- [ ] 정리 후보 파일 삭제 여부 최종 결정
- [ ] 최소 실행 테스트(개발자 UI + 사용자 Web UI) 완료
