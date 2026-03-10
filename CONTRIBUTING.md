# Contributing

이 저장소는 직접 `upstream`에 push하지 않고, 개인 fork의 브랜치로 올린 뒤 PR로 반영하는 방식으로 관리합니다.

## 기본 원칙

- `upstream`은 팀 저장소
- `origin`은 본인 fork
- 작업은 브랜치에서 진행
- 변경 이유와 테스트 방법을 남김

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
