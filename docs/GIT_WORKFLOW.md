# Git Workflow

## English

### Remotes

Recommended setup:

- `origin`: your fork
- `upstream`: team repository

```bash
git remote -v
```

### First-time fork setup

```bash
git remote add origin git@github.com:<YOUR_ID>/bartender-robot.git
git remote set-url upstream https://github.com/RobotTender/bartender-robot.git
```

### Daily flow

```bash
git fetch upstream
git switch -c feature/<short-name>
# work
git add .
git commit -m "feat: <summary>"
git push -u origin feature/<short-name>
```

Create a PR from your fork branch into the team target branch.

### Rules

- Do not push directly to `upstream`.
- Keep one logical change per PR when possible.

## Korean (한국어)

### 원격 저장소 구성

권장 구성:

- `origin`: 개인 fork
- `upstream`: 팀 저장소

```bash
git remote -v
```

### 최초 fork 설정

```bash
git remote add origin git@github.com:<YOUR_ID>/bartender-robot.git
git remote set-url upstream https://github.com/RobotTender/bartender-robot.git
```

### 일상 작업 흐름

```bash
git fetch upstream
git switch -c feature/<short-name>
# 작업
git add .
git commit -m "feat: <summary>"
git push -u origin feature/<short-name>
```

개인 fork 브랜치에서 팀 대상 브랜치로 PR을 생성합니다.

### 운영 규칙

- `upstream` 직접 push 금지
- 가능하면 PR 하나에 논리 변경 한 단위를 유지
