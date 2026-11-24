# 랜덤 맵 주행 요약

- 실행: 2026-11-24 22:10:58 · 소요 2.7분
- **시드 1457145442** · 세트 general200, official_like50, hills60, stress50 · 뽑기 balanced · 맵 6개
- 재현: `python3 experiments/eval/random_drive.py 6 --seed 1457145442 --suites general200,official_like50,hills60,stress50 --mode balanced` (같은 follower, LF_PARAMS)
- follower `experiments/user_base/run_user.py` · LF_PARAMS `/ws/src/line_follower/config/rule.yaml`
- bench: backend run_eval, --off 0.12, --limit 300 s, --stall 20 s, --jobs 3, 도메인 50–69, 파티션 `lf_l_rand_rule_r2_*`

## 결과

- **완주 0/6** (0%)
- 평균 진행률 58.3%
- 평균 랩타임(완주한 맵) -

## 맵별 결과

| # | 세트 | 맵 | 난이도 | 언덕 | 반전 | 코너 원 | 교차로 | 막대 | 끊김 | 물방울 | 상자 | 길이 m | 완주 | 진행률 | 랩 s | 결과 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | general200 | normal_1055 | normal | 1 | 0 | 0 | 1 | 2 | 2 | 0 | 0 | 17.746 | X | 55.9% | - | off line >0.12m at 9.91m |
| 2 | official_like50 | hard_5036_m | hard | 1 | 1 | 2 | 0 | 2 | 3 | 0 | 0 | 17.352 | X | 12.6% | - | off line >0.12m at 2.08m |
| 3 | stress50 | hard_7018 | hard | 1 | 0 | 2 | 0 | 4 | 3 | 1 | 0 | 18.392 | X | 96.7% | - | stalled at 17.79m |
| 4 | stress50 | hard_7037_m | hard | 1 | 1 | 2 | 0 | 3 | 3 | 2 | 0 | 19.322 | X | 44.5% | - | off line >0.12m at 8.57m |
| 5 | general200 | hard_1059 | hard | 1 | 0 | 0 | 0 | 1 | 2 | 0 | 0 | 15.894 | X | 92.6% | - | stalled at 14.72m |
| 6 | hills60 | easy_9024_m | easy | 3 | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 15.646 | X | 47.3% | - | off line >0.12m at 7.44m |

## 실패 원인

- **선 이탈** 4건: normal_1055 (9.91 m, 56%), hard_5036_m (2.08 m, 13%), hard_7037_m (8.57 m, 44%), easy_9024_m (7.44 m, 47%)
- **정지(진행 없음)** 2건: hard_7018 (17.79 m, 97%), hard_1059 (14.72 m, 93%)
  - 노드가 종점 T를 봤다고 판단하고 스스로 멈춤(follower 로그 'reached finish'): hard_7018 (주행 18.36 m, 실제 끝까지 0.60 m 남음), hard_1059 (주행 14.80 m, 실제 끝까지 1.18 m 남음). finish_min_dist 이후 보이는 T자·막대·곁가지를 종점으로 본 조기 종점 판정이다.

## 세트별·요소별 완주율

### 세트

| 세트 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| hills60 | 1 | 0 | 0% | 47.3% | - |
| stress50 | 2 | 0 | 0% | 70.6% | - |
| general200 | 2 | 0 | 0% | 74.2% | - |
| official_like50 | 1 | 0 | 0% | 12.6% | - |

### 난이도

| 난이도 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| easy | 1 | 0 | 0% | 47.3% | - |
| hard | 4 | 0 | 0% | 61.6% | - |
| normal | 1 | 0 | 0% | 55.9% | - |

### 언덕 수

| 언덕 수 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 1 | 5 | 0 | 0% | 60.5% | - |
| 3 | 1 | 0 | 0% | 47.3% | - |

### 좌우 반전

| 좌우 반전 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 3 | 0 | 0% | 81.7% | - |
| 1 | 3 | 0 | 0% | 34.8% | - |

### 코너 원

| 코너 원 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 3 | 0 | 0% | 65.3% | - |
| 2 | 3 | 0 | 0% | 51.3% | - |

### 연결 교차로

| 연결 교차로 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 5 | 0 | 0% | 58.7% | - |
| 1 | 1 | 0 | 0% | 55.9% | - |

### 교차 막대+곁가지

| 교차 막대+곁가지 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 1 | 0 | 0% | 47.3% | - |
| 1 | 1 | 0 | 0% | 92.6% | - |
| 2 | 2 | 0 | 0% | 34.2% | - |
| 3 | 1 | 0 | 0% | 44.5% | - |
| 4 | 1 | 0 | 0% | 96.7% | - |

### 끊긴 길

| 끊긴 길 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 1 | 0 | 0% | 47.3% | - |
| 2 | 2 | 0 | 0% | 74.2% | - |
| 3 | 3 | 0 | 0% | 51.3% | - |

### 물방울 원

| 물방울 원 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 4 | 0 | 0% | 52.1% | - |
| 1 | 1 | 0 | 0% | 96.7% | - |
| 2 | 1 | 0 | 0% | 44.5% | - |

### 상자

| 상자 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 6 | 0 | 0% | 58.3% | - |

## 부록: worlds/summarize_suite.py 출력

```
runs 6  (infrastructure errors 0 excluded)
ALL                      0/6     0.0%  mean  58.3%  lap      -    hits 0
  difficulty=easy        0/1     0.0%  mean  47.3%  lap      -    hits 0
  difficulty=hard        0/4     0.0%  mean  61.6%  lap      -    hits 0
  difficulty=normal      0/1     0.0%  mean  55.9%  lap      -    hits 0
  hills=1                0/5     0.0%  mean  60.5%  lap      -    hits 0
  hills=3                0/1     0.0%  mean  47.3%  lap      -    hits 0
  mirror=0               0/3     0.0%  mean  81.7%  lap      -    hits 0
  mirror=1               0/3     0.0%  mean  34.8%  lap      -    hits 0
  loop_corners=0         0/3     0.0%  mean  65.3%  lap      -    hits 0
  loop_corners=2         0/3     0.0%  mean  51.3%  lap      -    hits 0
  detours=0              0/5     0.0%  mean  58.7%  lap      -    hits 0
  detours=1              0/1     0.0%  mean  55.9%  lap      -    hits 0
  bars=0                 0/1     0.0%  mean  47.3%  lap      -    hits 0
  bars=1                 0/1     0.0%  mean  92.6%  lap      -    hits 0
  bars=2                 0/2     0.0%  mean  34.2%  lap      -    hits 0
  bars=3                 0/1     0.0%  mean  44.5%  lap      -    hits 0
  bars=4                 0/1     0.0%  mean  96.7%  lap      -    hits 0
  gaps=0                 0/1     0.0%  mean  47.3%  lap      -    hits 0
  gaps=2                 0/2     0.0%  mean  74.2%  lap      -    hits 0
  gaps=3                 0/3     0.0%  mean  51.3%  lap      -    hits 0
  small_loops=0          0/4     0.0%  mean  52.1%  lap      -    hits 0
  small_loops=1          0/1     0.0%  mean  96.7%  lap      -    hits 0
  small_loops=2          0/1     0.0%  mean  44.5%  lap      -    hits 0
  obstacles=0            0/6     0.0%  mean  58.3%  lap      -    hits 0
worst maps:
  hard_5036_m.sdf                          12.6%  off line >0.12m at 2.08m
  hard_7037_m.sdf                          44.5%  off line >0.12m at 8.57m
  easy_9024_m.sdf                          47.3%  off line >0.12m at 7.44m
  normal_1055.sdf                          55.9%  off line >0.12m at 9.91m
  hard_1059.sdf                            92.6%  stalled at 14.72m
  hard_7018.sdf                            96.7%  stalled at 17.79m
wrote /ws/experiments/results/situation/rule_r2/random/by_feature.png
```

![요소별 완주율](by_feature.png)

