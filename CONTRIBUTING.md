# Contributing

현재 협업 방식은 운영 중 조정 단계이며, 아래 내용은 임시 가이드입니다.

## 기본 원칙

- 작업 단위는 작게 나누고, 변경 이유와 테스트 방법을 기록합니다.
- 기능 변경 시 문서(`README`, `docs/*`)를 같이 갱신합니다.
- 하드웨어/환경 의존값(IP, 시리얼, 캘리브레이션 경로)은 코드 하드코딩 대신 설정으로 관리합니다.

## 문서

- Git 연결/업로드 절차:
  - [docs/GIT_WORKFLOW.md](docs/GIT_WORKFLOW.md)
- 배포 절차:
  - [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)

## 브랜치 규칙

- 기능: `feature/<name>`
- 수정: `fix/<name>`
- 긴급 수정: `hotfix/<name>`

## 커밋 권장 형식

예시:

```bash
git commit -m "feat: split calibration metadata pipeline"
git commit -m "fix: stop calibration publisher race on shutdown"
git commit -m "docs: add deployment guide"
```

## PR에 포함할 내용

- 왜 바꿨는지
- 무엇을 바꿨는지
- 어떻게 테스트했는지
- 배포 시 추가 작업이 필요한지

## 주의

- Doosan vendor patch가 필요한 변경은 문서에 같이 남깁니다.
- calibration, model path, robot IP 같은 현장 의존값은 PR 설명에 명시합니다.
