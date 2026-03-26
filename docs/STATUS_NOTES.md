# Current Status Notes

Last updated: 2026-03-26

## English

- Documentation has been normalized to bilingual format: English first, Korean second.
- Runtime entrypoint remains `run_bartender.py` + `launch/system_launch.py`.
- Sequence control authority remains in backend (`BartenderSequenceManager`).
- User web UI process starts by launch default, but ordering requires explicit enable flag.

## Korean (한국어)

- 문서를 영문 우선, 한글 후순서의 이중 언어 포맷으로 정리했습니다.
- 런타임 엔트리포인트는 `run_bartender.py` + `launch/system_launch.py`를 유지합니다.
- 시퀀스 제어 권한은 백엔드(`BartenderSequenceManager`)에 유지됩니다.
- 사용자 웹 UI 프로세스는 기본 실행되지만 주문 진입은 활성화 플래그가 필요합니다.
