## 변경 내용
<!-- 무엇을, 왜 바꿨는지 2~3줄. 관련 규칙: docs/decision_rules.md, docs/line_detection_rules.md -->

## 변경 범위
- [ ] 판단·제어 (`line_follower_navigation`, `line_follower_control`, `line_follower_mission`)
- [ ] 라인 검출 (`line_follower_perception`)
- [ ] 실행·설정 (`line_follower_bringup`, `Dockerfile`)
- [ ] 개발 도구·문서만 (`tools/`, `docs/`) — 아래 성능 항목 생략 가능

## 자동 검사 (CI가 확인)
- [ ] `static` — 제공 월드·모델 해시, `/world/`·`set_pose` 미사용
- [ ] `deps` — 모든 rosdep 키 해결, 대회용 패키지가 개발용 패키지(GCS·tests)에 의존하지 않음
- [ ] `build-test` — `package.xml`에 선언한 의존성만으로 빌드 + 단위 테스트
- [ ] `competition package build` — Dockerfile과 같은 조건(대회 패키지만, Release)으로 빌드
- [ ] 새 의존성은 `package.xml`에 선언했음 (CI에 수동 설치를 추가하지 않음)

## 성능 검사 (Gazebo, lab 환경 — 주행·검출 코드를 바꿨으면 필수)
`tools/lab_env.sh test` · `checkpoint-a` 통과 후, 깨끗한 복사본에서 **순차 실행**한 결과를 붙여 주세요.

| 항목 | 기준선 (main, v2) | 이 PR |
|---|---|---|
| `check-drive` 구간 4종 (hill/corner/curve/crossing) | 4/4 PASS | |
| 전체 주행 1 (`check-drive --only-full --full-runs 3`) | 9회 중 최고 76.8 % (12.06 m) | |
| 전체 주행 2 | 70 % 이상 도달 6/9 | |
| 전체 주행 3 | 첫 커브 이탈 약 1/3 | |
| 완주 시 시간 (s) | 미완주 | |
| `check-d05` (검출 변경 시) recall / p95 / max / 처리 p95 | 1.0 / ≤ 1.3 cm / 2.07 cm / ≤ 6.2 ms | |
| 물리 이상 (`check-physics`, wip/physics-check) | 이상 없음 | |

## 반영 기준 (모두 충족해야 merge)
- [ ] CI 전부 초록
- [ ] 성능이 **나빠지지 않음**: 최고 진행률 ≥ 기준선, 초반 실패 비율 증가 없음, 검출 정확도 기준 유지
- [ ] 안전 규칙 불변: 20초 대기, KILL, 입력 끊김 정지, 종료 시 0 명령, `safety_guard` 단독 `/cmd_vel`
- [ ] 런타임 코드에 트랙 좌표·정답·`/world/` 없음
- [ ] 이탈 판정(0.08 m / 0.6 s) 등 검사 기준을 완화하지 않음
