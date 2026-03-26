# Contributing

## English

### Principles

- Keep changes small and reviewable.
- Update docs together with behavior changes.
- Do not hardcode site-specific values (robot IP, serials, calibration files).

### Branch naming

- `feature/<name>`
- `fix/<name>`
- `hotfix/<name>`

### Commit style (recommended)

```bash
git commit -m "feat: add sequence precheck api"
git commit -m "fix: guard user frontend order-start flag"
git commit -m "docs: refresh deployment guide"
```

### PR checklist

- Why the change is needed
- What behavior changed
- How it was tested
- Any deployment impact

### References

- `docs/GIT_WORKFLOW.md`
- `docs/DEPLOYMENT.md`
- `docs/VENDOR_PATCHES.md`

## Korean (한국어)

### 기본 원칙

- 변경 단위를 작게 유지하고 리뷰 가능하게 만듭니다.
- 동작 변경 시 문서를 함께 업데이트합니다.
- 현장 의존값(로봇 IP, 시리얼, 캘리브레이션 파일)은 하드코딩하지 않습니다.

### 브랜치 규칙

- `feature/<name>`
- `fix/<name>`
- `hotfix/<name>`

### 커밋 형식(권장)

```bash
git commit -m "feat: add sequence precheck api"
git commit -m "fix: guard user frontend order-start flag"
git commit -m "docs: refresh deployment guide"
```

### PR 체크리스트

- 왜 필요한 변경인지
- 무엇이 바뀌는지
- 어떻게 검증했는지
- 배포 영향이 있는지

### 참고 문서

- `docs/GIT_WORKFLOW.md`
- `docs/DEPLOYMENT.md`
- `docs/VENDOR_PATCHES.md`
