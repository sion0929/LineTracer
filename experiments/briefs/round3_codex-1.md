# Codex-1 (룰 기반, 사용자 패키지 기반) — 3라운드

`git merge dev/two-track`을 먼저 해라. 최신 내용은 팀 분업 틀(`src/line_follower`의 perception/control 플러그인), 맵 생성기와 테스트 세트, bench 도메인 수정이다.

**기준 측정 (리드, `rule` 모드):** hills60 + official_like50에서 완주 3/108, 평균 29%. 가장 이른 실패는 hard 언덕 오르막과 내리막이다(experiments/results/lead/A_baseline_*). 리드가 만든 `perception/rule_plane.py`는 선 폭으로 지면 평면을 고르고 먼 구간을 잘라 낸다. 오프라인 개선은 확인했고 Gazebo 결과는 experiments/results/lead/hills60_ABC.md에 곧 나온다.

**할 일** (`perception/rule*.py`, `control/rule.py`, `config/rule*.yaml`만 고친다. 공용 파일은 리드에게 요청한다)
1. `control/rule.py`에서 공식 트랙 전용 설정을 없앤다. `speed_profile`(주행 거리 구간별 속도)과 `finish_min_dist`를 쓰지 않고, 보이는 경로 길이, 곡률, 제동 거리로 속도를 정한다. 종점은 앞쪽 선 재확인으로 판정한다.
2. 언덕 진입, 꼭대기, 내리막 끝에서 속도를 제한한다(IMU 피치 변화 감지). 내리막에서 실제 속도가 명령을 넘는 문제도 막는다.
3. 급코너 진입 감속과 회전 가속도 제한으로 미끄러짐을 줄인다(코너마다 지도 기준점이 13–25 cm 밀린다).
4. 평가는 bench로 한다: 공식 10회, `hills60`, `official_like50`(맵 목록은 experiments/worlds/suites/*/list.txt, 맵 파일은 make_suite.py로 다시 만든다. TEAM_GUIDE 4-1). 요약은 summarize_suite.py로 낸다.
5. 합격 기준: 공식 10회 중 9회 이상, hills60과 official_like50 완주율이 기준선(3/108)보다 확실히 높을 것.
6. 반복 실행은 DeepSeek 워커에게 맡긴다. 커밋은 track/a-follower에만 하고 push는 금지한다. 보고서는 experiments/reports/codex-1.md에 쓴다.
