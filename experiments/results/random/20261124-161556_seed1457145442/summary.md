# 랜덤 맵 주행 요약

- 실행: 2026-11-24 16:18:13 · 소요 2.3분
- **시드 1457145442** · 세트 general200, official_like50, hills60, stress50 · 뽑기 balanced · 맵 6개
- 재현: `python3 experiments/eval/random_drive.py 6 --seed 1457145442 --suites general200,official_like50,hills60,stress50 --mode balanced` (같은 follower, LF_PARAMS)
- follower `experiments/user_base/run_user.py` · LF_PARAMS `/ws/src/line_follower/config/rule.yaml`
- bench: backend run_eval, --off 0.12, --limit 300 s, --stall 20 s, --jobs 3, 도메인 50–69, 파티션 `lf_p_drive_*`

## 결과

- **완주 0/6** (0%)
- 평균 진행률 40.4%
- 평균 랩타임(완주한 맵) -

## 맵별 결과

| # | 세트 | 맵 | 난이도 | 언덕 | 반전 | 코너 원 | 교차로 | 막대 | 끊김 | 물방울 | 상자 | 길이 m | 완주 | 진행률 | 랩 s | 결과 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | general200 | normal_1055 | normal | 1 | 0 | 0 | 1 | 2 | 2 | 0 | 0 | 17.746 | X | 21.1% | - | off line >0.12m at 3.77m |
| 2 | official_like50 | hard_5036_m | hard | 1 | 1 | 2 | 0 | 2 | 3 | 0 | 0 | 17.352 | X | 5.6% | - | off line >0.12m at 1.05m |
| 3 | stress50 | hard_7018 | hard | 1 | 0 | 2 | 0 | 4 | 3 | 1 | 0 | 18.392 | X | 96.8% | - | stalled at 17.81m |
| 4 | stress50 | hard_7037_m | hard | 1 | 1 | 2 | 0 | 3 | 3 | 2 | 0 | 19.322 | X | 5.7% | - | off line >0.12m at 1.13m |
| 5 | general200 | hard_1059 | hard | 1 | 0 | 0 | 0 | 1 | 2 | 0 | 0 | 15.894 | X | 92.6% | - | stalled at 14.72m |
| 6 | hills60 | easy_9024_m | easy | 3 | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 15.646 | X | 20.3% | - | off line >0.12m at 3.20m |

## 실패 원인

- **선 이탈** 4건: normal_1055 (3.77 m, 21%), hard_5036_m (1.05 m, 6%), hard_7037_m (1.13 m, 6%), easy_9024_m (3.20 m, 20%)
- **정지(진행 없음)** 2건: hard_7018 (17.81 m, 97%), hard_1059 (14.72 m, 93%)
  - 노드가 종점 T를 봤다고 판단하고 스스로 멈춤(follower 로그 'reached finish'): hard_7018 (주행 17.97 m, 실제 끝까지 0.59 m 남음), hard_1059 (주행 14.77 m, 실제 끝까지 1.18 m 남음). finish_min_dist 이후 보이는 T자·막대·곁가지를 종점으로 본 조기 종점 판정이다.

## 세트별·요소별 완주율

### 세트

| 세트 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| hills60 | 1 | 0 | 0% | 20.3% | - |
| stress50 | 2 | 0 | 0% | 51.2% | - |
| general200 | 2 | 0 | 0% | 56.8% | - |
| official_like50 | 1 | 0 | 0% | 5.6% | - |

### 난이도

| 난이도 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| easy | 1 | 0 | 0% | 20.3% | - |
| hard | 4 | 0 | 0% | 50.2% | - |
| normal | 1 | 0 | 0% | 21.1% | - |

### 언덕 수

| 언덕 수 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 1 | 5 | 0 | 0% | 44.4% | - |
| 3 | 1 | 0 | 0% | 20.3% | - |

### 좌우 반전

| 좌우 반전 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 3 | 0 | 0% | 70.2% | - |
| 1 | 3 | 0 | 0% | 10.5% | - |

### 코너 원

| 코너 원 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 3 | 0 | 0% | 44.7% | - |
| 2 | 3 | 0 | 0% | 36.0% | - |

### 연결 교차로

| 연결 교차로 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 5 | 0 | 0% | 44.2% | - |
| 1 | 1 | 0 | 0% | 21.1% | - |

### 교차 막대+곁가지

| 교차 막대+곁가지 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 1 | 0 | 0% | 20.3% | - |
| 1 | 1 | 0 | 0% | 92.6% | - |
| 2 | 2 | 0 | 0% | 13.4% | - |
| 3 | 1 | 0 | 0% | 5.7% | - |
| 4 | 1 | 0 | 0% | 96.8% | - |

### 끊긴 길

| 끊긴 길 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 1 | 0 | 0% | 20.3% | - |
| 2 | 2 | 0 | 0% | 56.8% | - |
| 3 | 3 | 0 | 0% | 36.0% | - |

### 물방울 원

| 물방울 원 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 4 | 0 | 0% | 34.9% | - |
| 1 | 1 | 0 | 0% | 96.8% | - |
| 2 | 1 | 0 | 0% | 5.7% | - |

### 상자

| 상자 | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |
|---|---|---|---|---|---|
| 0 | 6 | 0 | 0% | 40.4% | - |

## 부록: worlds/summarize_suite.py 출력

```
runs 6  (infrastructure errors 0 excluded)
ALL                      0/6     0.0%  mean  40.4%  lap      -    hits 0
  difficulty=easy        0/1     0.0%  mean  20.3%  lap      -    hits 0
  difficulty=hard        0/4     0.0%  mean  50.2%  lap      -    hits 0
  difficulty=normal      0/1     0.0%  mean  21.1%  lap      -    hits 0
  hills=1                0/5     0.0%  mean  44.4%  lap      -    hits 0
  hills=3                0/1     0.0%  mean  20.3%  lap      -    hits 0
  mirror=0               0/3     0.0%  mean  70.2%  lap      -    hits 0
  mirror=1               0/3     0.0%  mean  10.5%  lap      -    hits 0
  loop_corners=0         0/3     0.0%  mean  44.7%  lap      -    hits 0
  loop_corners=2         0/3     0.0%  mean  36.0%  lap      -    hits 0
  detours=0              0/5     0.0%  mean  44.2%  lap      -    hits 0
  detours=1              0/1     0.0%  mean  21.1%  lap      -    hits 0
  bars=0                 0/1     0.0%  mean  20.3%  lap      -    hits 0
  bars=1                 0/1     0.0%  mean  92.6%  lap      -    hits 0
  bars=2                 0/2     0.0%  mean  13.4%  lap      -    hits 0
  bars=3                 0/1     0.0%  mean   5.7%  lap      -    hits 0
  bars=4                 0/1     0.0%  mean  96.8%  lap      -    hits 0
  gaps=0                 0/1     0.0%  mean  20.3%  lap      -    hits 0
  gaps=2                 0/2     0.0%  mean  56.8%  lap      -    hits 0
  gaps=3                 0/3     0.0%  mean  36.0%  lap      -    hits 0
  small_loops=0          0/4     0.0%  mean  34.9%  lap      -    hits 0
  small_loops=1          0/1     0.0%  mean  96.8%  lap      -    hits 0
  small_loops=2          0/1     0.0%  mean   5.7%  lap      -    hits 0
  obstacles=0            0/6     0.0%  mean  40.4%  lap      -    hits 0
worst maps:
  hard_5036_m.sdf                          5.6%  off line >0.12m at 1.05m
  hard_7037_m.sdf                          5.7%  off line >0.12m at 1.13m
  easy_9024_m.sdf                          20.3%  off line >0.12m at 3.20m
  normal_1055.sdf                          21.1%  off line >0.12m at 3.77m
  hard_1059.sdf                            92.6%  stalled at 14.72m
  hard_7018.sdf                            96.8%  stalled at 17.81m
wrote /ws/experiments/results/random/20261124-161556_seed1457145442/by_feature.png
```

![요소별 완주율](by_feature.png)

