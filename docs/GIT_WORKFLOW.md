# Git Workflow

## 현재 상태

현재 로컬 저장소 상태:

- 현재 브랜치: `bartender-robot`
- 연결된 원격: `upstream = https://github.com/RobotTender/bartender-robot.git`
- 아직 `origin`은 연결되지 않음

즉, 지금은 팀 저장소만 연결돼 있고 본인 포크 저장소 연결은 직접 해야 합니다.

## 사용자가 해야 하는 것

### 1. GitHub에서 본인 fork 만들기

브라우저에서 아래 저장소를 fork 합니다.

- `https://github.com/RobotTender/bartender-robot`

예시 fork 주소:

- `https://github.com/<YOUR_ID>/bartender-robot`

### 2. SSH 키 등록

이미 등록돼 있지 않으면:

```bash
ssh-keygen -t ed25519 -C "YOUR_EMAIL"
cat ~/.ssh/id_ed25519.pub
```

출력된 공개키를 GitHub 계정의 SSH Keys에 등록합니다.

### 3. Git 사용자 정보 설정

```bash
git config --global user.name "YOUR_NAME"
git config --global user.email "YOUR_EMAIL"
git config --global pull.rebase true
```

### 4. `origin` remote 연결

SSH 권장:

```bash
cd <repo-root>
git remote add origin git@github.com:<YOUR_ID>/bartender-robot.git
```

이미 `origin`이 있으면:

```bash
git remote set-url origin git@github.com:<YOUR_ID>/bartender-robot.git
```

확인:

```bash
git remote -v
```

원하는 상태:

```text
origin    git@github.com:<YOUR_ID>/bartender-robot.git
upstream  https://github.com/RobotTender/bartender-robot.git
```

## 첫 업로드 절차

이 저장소는 아직 초기 커밋 전 상태일 수 있으니, 아래 순서로 올리면 됩니다.

```bash
cd <repo-root>
git status
git add .
git commit -m "chore: initialize bartender robot runtime repository"
git push -u origin bartender-robot
```

## 이후 작업 절차

### upstream 최신 반영

```bash
git fetch upstream
```

### 새 작업 브랜치 생성

기능 작업:

```bash
git switch -c feature/<short-name>
```

버그 수정:

```bash
git switch -c fix/<short-name>
```

### 작업 후 업로드

```bash
git add .
git commit -m "feat: <summary>"
git push -u origin <branch-name>
```

## PR 생성

GitHub에서:

- source: `YOUR_ID/<branch-name>`
- target: `RobotTender/main`

또는 upstream 기본 브랜치가 `bartender-robot`이면 그 브랜치로 보냅니다. 실제 대상 브랜치는 GitHub에서 확인해야 합니다.

## 추천 브랜치 전략

- 저장소 기준 기본 작업 브랜치: `bartender-robot`
- 기능 작업: `feature/...`
- 수정 작업: `fix/...`
- 급한 수정: `hotfix/...`

## 원격 연결 예시 전체

```bash
cd <repo-root>
git remote add origin git@github.com:<YOUR_ID>/bartender-robot.git
git remote set-url upstream https://github.com/RobotTender/bartender-robot.git
git remote -v
git add .
git commit -m "chore: initialize runtime repository"
git push -u origin bartender-robot
```

## 확인 포인트

- `origin`은 본인 fork
- `upstream`은 팀 저장소
- 직접 `upstream`에 push하지 않음
- 보통은 `origin`으로 올리고 PR 생성
