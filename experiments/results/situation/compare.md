# 규칙 모드 대비 비교 (보류 평가 세트, 반복 합산)

같은 하네스(bench run_eval, --off 0.12, --limit 300, --stall 20, run_user.py + LF_PARAMS, 3개 동시, 도메인 50-69). 핵심 세트 = 공식 트랙 3회 + 시드 0-19 + random_drive 6 --seed 1457145442 = 반복당 29회.

| 모드 | 반복(태그) | 묶음 | 회차 | 완주 | 평균 진행률 | 평균 랩 s | 잘못된 완주 | 정상 완주 판정 | 완주했지만 판정 없음 |
|---|---|---|---|---|---|---|---|---|---|
| rule | rule,rule_r2 | official | 6 | 5 | 92.7% | 63.8 | 0 | 5 | 0 |
| rule | rule,rule_r2 | seeds | 40 | 8 | 49.8% | 67.4 | 1 | 8 | 0 |
| rule | rule,rule_r2 | random | 12 | 0 | 46.1% | - | 3 | 0 | 0 |
| rule | rule,rule_r2 | **합계(핵심)** | 58 | 13 | 53.4% | 66.0 | 4 | 13 | 0 |
| rule | rule,rule_r2 | finishgate | 12 | 2 | 58.9% | 54.0 | 1 | 1 | 1 |
| playbook | playbook_v0,playbook_v0_r2 | official | 6 | 5 | 85.8% | 64.2 | 0 | 5 | 0 |
| playbook | playbook_v0,playbook_v0_r2 | seeds | 40 | 12 | 51.8% | 69.5 | 0 | 12 | 0 |
| playbook | playbook_v0,playbook_v0_r2 | random | 12 | 2 | 36.1% | 71.2 | 0 | 2 | 0 |
| playbook | playbook_v0,playbook_v0_r2 | **합계(핵심)** | 58 | 19 | 52.1% | 68.3 | 0 | 19 | 0 |
| playbook | playbook_v0,playbook_v0_r2 | finishgate | 12 | 3 | 55.8% | 59.4 | 0 | 3 | 0 |
| playbook_tuned | playbook | official | 3 | 2 | 72.2% | 67.2 | 0 | 2 | 0 |
| playbook_tuned | playbook | seeds | 20 | 2 | 44.2% | 76.0 | 0 | 2 | 0 |
| playbook_tuned | playbook | random | 6 | 0 | 31.9% | - | 0 | 0 | 0 |
| playbook_tuned | playbook | **합계(핵심)** | 29 | 4 | 44.5% | 71.6 | 0 | 4 | 0 |
| playbook_tuned | playbook | finishgate | 6 | 2 | 60.9% | 74.9 | 0 | 2 | 0 |

## 둘 다 완주한 맵의 랩(쌍 비교, 맵별 평균)

| 비교 | 맵 수 | rule | playbook |
|---|---|---|---|
| rule vs playbook | 5 (easy_3, hard_14, hard_15, line_track, normal_11) | 66.9 | 66.7 |

## 잘못된 완주(노드가 종점을 확정했지만 완주하지 못한 회차)

- rule: 5건: hard_16 (seeds, 진행 14.717 / 16.25 m, T 확정 주행 14.5 m), hard_1059 (random, 진행 14.721 / 15.894 m, T 확정 주행 14.01 m), normal_26_x4 (finishgate, 진행 15.102 / 16.043 m, T 확정 주행 14.54 m), hard_7018 (random, 진행 17.79 / 18.392 m, T 확정 주행 17.55 m), hard_1059 (random, 진행 14.719 / 15.894 m, T 확정 주행 14.02 m)
- playbook: 0건
- playbook_tuned: 0건
