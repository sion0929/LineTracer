# 판단 학습 기록: 상황 분류기 + 상황별 대처표 (playbook)

> 역할: 판단(TEAM_ROLES §3). 근거 규칙: `docs/decision_rules.md` §3(구간 패턴 대처표 + Gazebo 자동 튜닝, 기준선보다 나쁘면 채택하지 않음).
> 사용자 결정(2026-11-25): **학습된 상황 분류기 + 상황별 대처표**. 생성 맵의 정답으로 자동 라벨링하고, CPU에서 도는 작은 분류기를 학습하고, 상황별 파라미터를 Gazebo에서 튜닝한다. 같은 시험에서 규칙 모드보다 나을 때만 채택한다.
> 작성: 2026-11-25, 브랜치 `sion`(커밋하지 않음).


## 1. 한눈에

| 항목 | 내용 |
|---|---|
| 런타임 코드 | `src/line_follower/line_follower/situation/`(특징·모델), `control/playbook.py`(`controller: playbook`), `config/playbook.yaml`(대처표), `config/situation_model.yaml`(분류기), `config/playbook_rule.yaml`(= `rule.yaml` + `controller: playbook`) |
| 개발 도구(런타임 금지) | `experiments/situation/` — `label.py`(정답 라벨), `synth.py`(합성 재생), `collect_node.py`·`collect.py`(Gazebo 수집·데이터셋), `train.py`(학습·지표), `tune.py`(좌표 하강 튜닝), `evaluate.py`(보류 세트 평가·비교) |
| 결과 | `experiments/results/situation/` — `rule/`, `playbook*/`(평가), `model/metrics.md`(분류기 지표), `tuning/`(튜닝 전 시도), `compare.md` |
| 규칙 준수 | 런타임은 이번 주행의 카메라 선 지도·odom·IMU만 쓴다. 정답(맵 JSON, Gazebo 자세)은 `experiments/`의 라벨·수집·채점에서만 쓴다. 트랙 좌표 저장·teach & repeat 없음. 주행 거리 게이트(`finish_min_dist`)도 쓰지 않는다 |

## 2. 상황 분류 체계

`decision_rules.md` §3.2의 패턴과 생성기(`gen_world.py`) 요소를 합쳐 11개로 정했다. 순서가 곧 모델의 클래스 번호다(`situation/__init__.py`).
라벨은 로봇의 정답 자세에서 경로를 따라 앞쪽 창 **[s+0.05, s+0.85] m**(카메라가 보는 0.31–0.85 m와 지도에 남은 부분)을 보고 붙인다. 여러 개가 겹치면 표의 위쪽이 이긴다.

| 상황 | 정답 정의(`label.py`) | 대처표에서 바꿀 수 있는 것 |
|---|---|---|
| FINISH_T | 경로 끝이 0.90 m 안 | 종점 판정(아래 §5) |
| START_T | 경로 위가 아니고(역주행), 출발 T 막대를 0.9 m 안에서 마주봄 | 종점으로 받지 않음 |
| OFF | 경로에서 10 cm 넘게 벗어났거나 방향이 60° 넘게 어긋남 | 속도 상한 |
| LOOP_CORNER | 창이 작은 루프(코너 원, 물방울 원: 자기 교차 간격 ≤ 1.3 m)와 겹침 | 속도, 각속도, 전방 주시 거리 |
| CROSSING | 교차 막대, 큰 보조 루프, 연결 교차로(자기 교차 간격 > 1.3 m)의 교차점이 창 안 | 속도 |
| STUB | 한쪽 곁가지 뿌리가 창 안 | 속도 |
| GAP | 끊긴 길 구간이 창과 겹침 | 속도 |
| HILL | 경사로 위이거나 0.5 m 안에서 경사 시작 | 속도 |
| RIGHT_ANGLE | [s, s+0.8]에서 0.12 m 안에 방향이 60° 이상 바뀜(반지름 약 11 cm 미만) | 속도, 각속도, 전방 주시 거리 |
| ARC | [s, s+0.8]에서 방향 변화 30° 이상 | 속도, 횡가속 |
| STRAIGHT | 나머지 | 최고 속도 |

**T 후보 라벨**: 노드의 경로가 T자(`path_end == 't'`)로 끝날 때는 그 끝점을 정답 좌표로 옮겨 **그 점이 실제로 무엇인지**(`candidate_kind`)를 본다. 끝점이 실제 결승 막대(0.15 m 안)일 때만 FINISH_T다. 창 안에 결승이 있어도 후보가 결승 0.6 m 앞의 교차 막대면 CROSSING이 된다(무작위 맵 hard_7018처럼 결승 직전 막대에서 서는 경우를 학습시키기 위해).

## 3. 런타임 특징 (`situation/features.py`, 36개)

모두 노드가 이미 가진 것만 쓴다: odom 격자 선 지도(`LineMap.grid`)를 로봇 좌표로 잘라낸 패치(1 cm 칸, 앞 −0.10–0.90 m × 좌우 ±0.45 m), 노드가 추적한 경로(`node.path`, `node.path_end`), IMU 피치, 이번 주행의 출발 위치(odom).

| 묶음 | 개수 | 내용 |
|---|---|---|
| `occ_*` | 15 | 패치를 앞뒤 5칸 × 좌중우 3칸으로 나눈 선 비율 |
| `path_*` | 9 | 경로 길이, 끝 종류(open/t/len), 끝점 x·y, 순 회전각, 0.15 m 안 최대 회전각, 굽이(회전 합 − 순 회전) |
| `junc_*` | 4 | 경로 점 4 cm마다 반지름 7 cm 고리에서 본 곁가지(앞으로도 이어질 때만): 왼쪽 수, 오른쪽 수, 양쪽 수, 첫 갈래까지 거리 |
| `end_*` | 5 | 경로 끝(T 후보): 막대 왼쪽·오른쪽 팔 길이(최대 0.30 m), 막대 너머 선 비율, **지금 카메라가 막대 너머를 얼마나 보는지**(m), 로봇까지 거리 |
| `ctx_*` | 3 | IMU 피치(°), 출발 위치까지 거리(1.5 m 상한), 출발 위치를 바라보는 정도(cos) |

주행 거리(`travel`)는 넣지 않았다. 학습 맵 길이(15–19 m)를 외워 "14 m 이후면 종점"을 다시 배우는 것을 막기 위해서다. 출발 거리도 1.5 m에서 자른다(출발 T 구별용으로만).
측정 지연: 특징 0.5–0.6 ms(벡터화 후, 처음 구현은 1.1 ms), 분류기는 §4 참고.

## 4. 데이터와 분류기

### 4.1 맵 분할 (시드 ≥ 1000만 학습·튜닝, 0–39와 공식 트랙은 평가 전용)

| 묶음 | 맵 | 용도 |
|---|---|---|
| gz_train | 30개(general200 1000–1011, official_like50 5000–5007, stress50 7000–7005, hills60 9000–9003) | Gazebo 수집 → 학습 |
| gz_val | 12개(general200 1100–1103, official_like50 5040–5043, stress50 7040–7041, hills60 9050–9051) | Gazebo 수집 → **보류 지표 전용**(학습·모델 선택에 안 씀) |
| synth | 342개(위 4개 세트에서 gz_val과 아래 6개를 뺀 전부, gz_train 포함) | 합성 재생 → 학습 |
| 튜닝 | 12개(general200 1012–1019, official_like50 5008–5011) | §7 좌표 하강 |
| 제외 | normal_1055, hard_1059, hard_5036_m, hard_7018, hard_7037_m, easy_9024_m | `random_drive.py 6 --seed 1457145442` 평가 맵이라 어디에도 안 씀 |

### 4.2 Gazebo 수집 (`collect_node.py`, 2026-11-24)

- 패키지 노드를 그대로 돌리고(LF_PARAMS = `rule.yaml`) 5 Hz와 T 후보가 생길 때마다 로봇 좌표 지도 패치(비트 압축), 경로, 경로 끝, 피치, 시야 한계, 출발 위치, 노드 자세, Gazebo 정답 자세를 저장한다. 라벨은 나중에(`collect.py build`) 맵 JSON으로 붙인다.
- rule 모드는 첫 급코너에서 자주 이탈하므로 맵마다 **실제 출발 + 경로 30 %·55 %·끝 4 m 전의 곧고 평평한 지점에서 출발**하는 구간 월드를 만들어 맵 전체를 덮었다(로봇 시작 자세만 옮긴 SDF, 채점용 중심선은 그 점부터 자름).
- 종점 판정은 수집 때만 정답으로 한다(후보가 실제 끝 0.15 m 안이고 0.40 m 앞일 때 수락). 잘못된 T에서 멈추지 않아 구간 전체가 기록되고, 실제 T 접근도 충분히 남는다.
- bench `--off 0.2`, `--limit 150`, `--stall 15`, 3개 동시, 도메인 50–69.

| 묶음 | 회차 | 완주 | 인프라 오류 | 진행 합계 | 표본 |
|---|---|---|---|---|---|
| gz_train | 85 | 24 | 7 | 460.9 m | 18 246 |
| gz_val | 36 | 11 | 2 | 177.2 m | 6 310 |

### 4.3 합성 재생 (`synth.py`)

정답 경로를 따라 로봇을 움직이며(횡 흔들림 AR(1) 표준편차 약 1.5 cm, 급코너 근처 15 %로 6–20 cm 이탈, 방향 잡음 1.5°) 매 프레임(1.2 cm) 정답 자세에서 본 칠한 선(끊긴 길 제외, T 막대·보조 루프·곁가지·교차 막대 포함)을 노드의 BEV 격자에 그리고, **odom 자세**로 실제 `LineMap.update`에 쓴다. odom 드리프트는 느린 랜덤 워크 + 급코너마다 진행 방향으로 5–25 cm 점프(실측 13–25 cm 밀림, TEAM_GUIDE §7). 인지 잡음은 선 굵기 ±5 mm, 빈 조각, 가짜 점, 먼 줄 얇아짐, 3 % 프레임 누락, 경사·흔들림 때 시야 0.50 m. 노드의 `prune/acquire/replan`을 그대로 옮겨 경로와 끝 종류도 패키지의 `LineMap.trace`에서 나온다.
변형: 정방향 전체, 중간 출발 정방향, 출발 T로 되돌아오는 역주행(START_T만 남김), **곁가지·교차 막대를 따라 길 쪽으로 가는 길 잃은 로봇**(막대가 길 자체인 T: 긴 팔, 너머 빈칸 → OFF). 마지막 변형은 safety 5b의 "선을 놓친 뒤 루프 교차에서 정지"를 학습시키기 위한 음성 표본이다.

| 파일 | 표본 | 비고 |
|---|---|---|
| `data/synth.npz` | 168 989 | 342맵, 맵당 약 35 s(4 워커, 단일 스레드) |
| `data/synth_markings.npz` | 6 230 | 길 잃은 접근(OFF 6 186) |

실패한 시도(지우지 않음): 첫 합성은 흔들림 모형이 잘못돼(OU 적분, 표준편차 약 14 cm) 표본의 대부분이 OFF였고, 재생한 경로가 로봇 뒤에 남아 같은 T 후보가 반복됐다. AR(1)로 바꾸고 "경로가 로봇에서 15 cm 넘게 떨어지거나 뒤로 향하면 다시 잡기"를 넣어 고쳤다. 첫 대량 합성은 OpenCV·BLAS 다중 스레드로 부하 48(16코어)을 만들어 Gazebo 수집을 굶겨서 중단하고 워커당 1 스레드로 다시 돌렸다.

### 4.4 분류기 (`train.py`)

- 후보: numpy MLP 36→64→32→11, numpy MLP 36→96→48→11(`mlp_wide`), OpenCV RTrees(25그루, 깊이 8). 표준화 + 클래스 가중(1/√빈도) + T 후보 ×2 + Gazebo 표본 ×3, Adam, 코사인 학습률, 40 에포크.
- 선택: 학습 데이터 안에서 **맵 단위 20 % 분할**(gz_val은 쓰지 않음). 1 MB 미만이고, 종점 판정 재생에서 잘못된 확정이 0이 되는 P(FINISH_T) 문턱이 있는 후보 중 Gazebo 표본 macro F1 최대.

| 후보 | macro F1(분할 전체) | macro F1(분할 Gazebo) | 크기 | 잘못된 확정 0인 최저 문턱 |
|---|---|---|---|---|
| mlp | 0.744 | **0.631** | 90 KB | 없음(0.4–0.9 모두 1건) |
| **mlp_wide (선택)** | 0.748 | 0.626 | 162 KB | **0.6** |
| rtrees | 0.694 | 0.554 | 1.69 MB(크기 초과) | 0.6 |

첫 학습(선택 기준 "잘못된 확정 수 우선")은 RTrees를 골랐지만 1.6 MB에 Gazebo F1도 낮아 기준을 위처럼 바꿨다.

**보류 지표(gz_val 12맵, 6 310 표본, 실제 Gazebo)** — 전체 표는 `experiments/results/situation/model/metrics.md`

| 상황 | 표본 | 정밀도 | 재현율 | T 후보 표본 | T 후보 정밀도 | T 후보 재현율 |
|---|---|---|---|---|---|---|
| STRAIGHT | 326 | 0.51 | 0.53 | 7 | - | 0.00 |
| ARC | 829 | 0.71 | 0.75 | 0 | - | - |
| RIGHT_ANGLE | 462 | 0.60 | 0.55 | 2 | - | 0.00 |
| CROSSING | 1 402 | 0.80 | 0.70 | 207 | 0.71 | **0.87** |
| LOOP_CORNER | 680 | 0.65 | 0.73 | 167 | 0.86 | 0.78 |
| STUB | 95 | 0.42 | **0.55** | 1 | 0.00 | 0.00 |
| GAP | 307 | 0.20 | 0.30 | 12 | 0.89 | 0.67 |
| HILL | 981 | 0.83 | 0.82 | 19 | 1.00 | 0.84 |
| **FINISH_T** | 465 | **0.977** | 0.918 | 317 | **0.984** | 1.000 |
| START_T | 0 | - | - | 0 | - | - |
| OFF | 763 | 0.60 | 0.55 | 105 | 0.74 | 0.51 |

정확도 0.685, macro F1 0.632. 종점 판정 재생(T 후보 순서에 `confirm_finish` 규칙 적용): T 후보가 생긴 26회차 중 **잘못된 확정 0**, 실제 종점 후보가 있던 11회차 중 9회 올바르게 확정(나머지 2회는 수집 노드가 0.40 m에서 정답으로 먼저 확정해 3프레임이 모이지 않음). P(FINISH_T)만 보면(argmax) 종점이 아닌 T 후보 4개가 FINISH_T로 나왔는데 모두 "막대 너머 시야 0 m"(흔들림으로 시야 0.50 m)였다. §5의 시야 조건이 이것을 막는다.
START_T는 Gazebo 데이터에 없다(rule 수집 주행이 출발 T로 되돌아간 적 없음). 합성 표본 10 152개로만 배웠고, 단위 시험(출발 위치를 마주보는 T 거부)으로만 확인했다.

- 모델 파일 `config/situation_model.yaml` 162 KB(cv2.FileStorage, `config/*.yaml` 설치 규칙으로 배포). 예측 0.02 ms, 특징 0.5–1.2 ms(부하에 따라), 합계 2 ms 미만(단위 시험으로 고정).
- 시험 고정물 `test/data/situation_fixture.npz` 80 KB: gz_val의 실제 Gazebo 표본 250개(상황별 25개, 가능하면 절반은 T 후보). 상황별 정확도 하한과 "종점 아닌 T 후보를 종점 판정 한 프레임이 받아들이지 않음"을 시험한다.


## 5. 종점 판정 (`PlaybookController.confirm_finish`, 거리 게이트 대체)

노드는 추적 경로가 T자로 끝날 때마다 `confirm_finish`를 부른다. 플레이북은 **None을 돌려주지 않으므로 `finish_min_dist`(14 m)는 더 이상 쓰이지 않는다.** 받아들이는 조건(모두 만족, `config/playbook.yaml`의 `finish`):

1. 분류기의 P(FINISH_T) ≥ `p_min`
2. 카메라 시야가 이미 막대 너머 `beyond_seen_min`(0.10 m) 이상을 보고 있음 — 시야 끝(0.85 m)에 걸친 교차 막대는 너머가 아직 안 보여 T와 구별되지 않는다(safety 5b의 원인)
3. 막대 너머 지도에 선이 거의 없음(`beyond_hit_max`)
4. 같은 자리(0.15 m 안)의 후보에서 `frames`번 연속으로 1–3을 만족

확정 전에는 T 후보까지 멈출 수 있는 속도(√(2·a·d) + v_min, a = `approach_decel`)로 제한한다. 한 번 확정하면 노드가 경로를 고정하므로(되돌릴 수 없음) 잘못된 확정을 가장 크게 본다.

## 6. 상황별 대처 (`compute`)

0.1 s마다 분류해 확률을 지수 평균(가중 0.5)하고, 최대 확률이 `min_confidence`(0.45) 이상이면 그 상황의 행을 적용한다. 행이 비면 rule과 완전히 같다(단위 시험으로 고정).
- `v_cap`: 속도 상한. 순수 추종 곡률을 지키도록 w도 같은 비율로 줄인다.
- `v_max`, `a_lat`, `a_dec`, `omega_max`, `lookahead_min`, `lookahead_gain`: 그 상황에서만 노드 파라미터를 바꿔 rule 제어기에 넘긴다(파라미터 프록시, `control/rule.py`는 수정하지 않음).
- 교차로 잠금·곁가지 무시는 따로 만들지 않았다. `LineMap.trace`가 이미 가장 곧은 갈래를 고르고, 생성기의 모든 교차(코너 원 포함)에서 정답이 직진이기 때문이다. 곁가지·교차를 종점으로 보지 않는 것은 §5가 맡는다.

## 7. 튜닝 기록

날짜 2026-11-25(컨테이너 시각 2026-11-24 21:12–21:48 UTC), 도구 `experiments/situation/tune.py`, 증거 `experiments/results/situation/tuning/<단계>/`(후보 YAML `configs/`, 주행 `bench/*/result.json`·`telemetry.png`, `summary.json`), 표 `tuning/summary.md`.

- **튜닝 맵** 12개: general200 1012–1019, official_like50 5008–5011(시드 ≥ 1000, 분류기 보류 맵·무작위 6맵과 겹치지 않음). 실제 출발 T에서 완주까지, bench run_eval, `--off 0.12`, LF_PARAMS = `playbook_rule.yaml`, 후보는 `-p playbook.config:=...`로 넘김, 3개 동시.
- **목적함수**(낮을수록 좋음, `decision_rules.md` §3.3): 회차마다 J = 시간(완주면 랩, 아니면 끝난 시점의 시뮬레이션 시간) + 500·max(0, 횡오차 p95 − 0.06 m) + 실패 패널티(200 s + 200 s·(1 − 진행 비율)). 맵 합계.
- **탐색 공간**: 좌표 하강. 1단계 상황별 속도 상한(`RIGHT_ANGLE.v_cap` 0.15, `ARC.v_cap` 0.18, `HILL.v_cap` 0.18, 빈 표), 2단계 `HILL.v_cap 0.18` 위에서 `ARC.v_cap` {없음, 0.18, 0.15}, 3단계 재현(튜닝값 대 빈 표, 맵마다 2회). **시도 9개 설정, 주행 132회.**

| 단계 | 후보 | 회차 | 완주 | 평균 진행률 | J | 4 m 전 이탈 | 선택 |
|---|---|---|---|---|---|---|---|
| r1_caps | `RIGHT_ANGLE.v_cap=0.15` | 12 | 0 | 26.6% | 4772 | 9 | |
| r1_caps | `ARC.v_cap=0.18` | 12 | 2 | 34.8% | 4122 | 8 | |
| r1_caps | `HILL.v_cap=0.18` | 12 | 2 | 37.3% | 4110 | 8 | ✓ |
| r1_caps | 빈 표 | 12 | 0 | 24.2% | 4759 | 9 | |
| r2_arc | + `ARC.v_cap=0.18` | 12 | 2 | 39.3% | 4062 | 7 | ✓ |
| r2_arc | + `ARC.v_cap=0.15` | 12 | 1 | 38.3% | 4347 | 7 | |
| r2_arc | `HILL.v_cap=0.18`만 | 12 | 2 | 31.4% | 4236 | 10 | |
| r3_repro | **튜닝값(HILL 0.18 + ARC 0.18)** | 24 | **4** | **38.6%** | **8213** | 15 | ✓ |
| r3_repro | 빈 표 | 24 | 2 | 25.7% | 9061 | 20 | |

**선택값**(`config/playbook.yaml`): `HILL: {v_cap: 0.18}`, `ARC: {v_cap: 0.18}`. 나머지 행은 비워 둠.
**해석과 한계**:
- 튜닝 맵에서 실패의 대부분(빈 표 36회 중 29회)은 첫 언덕 바로 뒤 첫 코너(2.7–3.6 m)다. 텔레메트리(예: `r1_caps/bench/01_04`, easy_1012_m)를 보면 원호 안에서 노드의 경로가 사라지고(속도 v_min, 각속도 0) 노드의 "경로 없음 → 0.35 m 직진" 동작으로 선을 벗어난다. 즉 원인은 인지(지도에서 원호를 잃음)이고, 경로가 없을 때는 `compute`가 불리지 않아 플레이북이 개입할 수 없다. 속도 상한은 원호에 천천히 들어가 이 상실을 덜 일으키는 정도의 효과다.
- 같은 설정도 회차마다 J가 ±150 정도 흔들린다(`HILL.v_cap=0.18`: 4110 → 4236). 그래서 3단계에서 맵마다 2회씩 재현했고, 세 단계 모두 같은 방향(튜닝값 6/36 완주·진행률 약 39 %, 빈 표 2/36·약 25 %)이었다. 여전히 표본이 작아 결론은 "유의미하게 나쁘지 않고 나아 보인다" 수준이다.
- **실패한 시도(지우지 않음)**: `RIGHT_ANGLE.v_cap=0.15`는 빈 표와 같았다(0/12). 분류기의 RIGHT_ANGLE 재현율이 0.55라 급코너 접근을 충분히 일찍 잡지 못하고, 실패 원인이 속도가 아니기 때문으로 본다. `ARC.v_cap=0.15`는 0.18보다 나빴다(1/12).
- 시도하지 않은 것: `omega_max`, `lookahead_*`, `a_lat`, CROSSING·LOOP_CORNER·STRAIGHT 행, 종점 문턱(`p_min`은 학습 쪽 분할에서 0.6으로 정했고 Gazebo에서 따로 튜닝하지 않음).


## 8. 보류 세트 평가와 채택 결정

증거: `experiments/results/situation/`의 `rule/`, `rule_r2/`, `playbook_v0/`, `playbook_v0_r2/`(대처표 비움 = 지금 배포 설정), `playbook/`(튜닝 대처표), 합산 표 `compare.md`, 반복 1 맵별 표 `compare_all.md`. 날짜 2026-11-25 02:03–07:24 KST(컨테이너 UTC 2026-11-24 17:03–22:24).

- **하네스(모든 모드 동일)**: `evaluate.py` → bench.py run_eval, `--off 0.12`, `--limit 300`, `--stall 20`, `experiments/user_base/run_user.py` + LF_PARAMS(`rule.yaml` 또는 `playbook_rule.yaml`), 3개 동시, 도메인 50–69, 파티션 `lf_l_*`. 인프라 오류 0건.
- **보류 세트(반복당 29회)**: 공식 트랙 3회, 생성 맵 시드 0–19(easy 0–6, normal 7–13, hard 14–19, 상자 0), `random_drive.py 6 --seed 1457145442`. 추가로 safety 5a·5b 맵(`short_t`, `normal_26_x4`, `normal_13_x4`, 각 2회, 합계에는 안 넣음).
- **지표**: 완주(진행 ≥ 끝 − 0.12 m), 평균 진행률, 완주 회차 평균 랩, **잘못된 완주** = 노드가 종점을 확정(`finish T detected`)했는데 완주하지 못한 회차.

### 8.1 합산 (반복 2회 = 58회)

| 모드 | 완주 | 평균 진행률 | 평균 랩 s | 잘못된 완주 | 정상 완주 판정 | 4 m 전 이탈 | 5a·5b 맵(12회) |
|---|---|---|---|---|---|---|---|
| rule (`rule.yaml`) | 13/58 | 53.4 % | 66.0 | **4** | 13 | 27 | 완주 2, 잘못된 완주 1, 판정 없이 통과 1 |
| **playbook (종점 판정, 대처표 비움)** | **19/58** | 52.1 % | 68.3 | **0** | 19 | 30 | 완주 3, 잘못된 완주 0 |
| playbook 튜닝 대처표(반복 1, 29회) | 4/29 | 44.5 % | 71.6 | 0 | 4 | 15/29 | 완주 2, 잘못된 완주 0 |

반복별(핵심 29회): rule 6 / 50.2 % / 잘못된 완주 2 → 7 / 56.7 % / 2, playbook 10 / 54.3 % / 0 → 9 / 49.9 % / 0.
**둘 다 완주한 맵의 쌍 비교 랩**(easy_3, hard_14, hard_15, line_track, normal_11): rule 66.9 s, playbook 66.7 s — 랩 손해 없음(합산 평균 랩 차이는 완주한 맵 구성이 달라서 생김).
**rule의 잘못된 완주 5건**(모두 재현됨): hard_1059 2회(14.72 / 15.89 m에서 정지, 큰 보조 루프 교차), hard_7018(17.79 / 18.39 m), hard_16(14.72 / 16.25 m), normal_26_x4(15.10 / 16.04 m, +자 교차). playbook은 같은 맵에서 0건이고 hard_1059·normal_26_x4는 완주했다. 완주한 19회 모두 실제 결승 T에서 종점을 확정했다(놓친 종점 0).

### 8.2 해석

- 확실한 이득은 **잘못된 완주 4 → 0**(5a·5b 맵 포함 5 → 0)이다. 이것은 구조적으로 재현된다(rule의 5건은 2회 반복에서 같은 자리).
- 완주 +6/58은 일부가 이 효과(종점 직전에 서던 회차)이고 나머지는 잡음이다. 4 m 전 이탈은 27 대 30인데, 첫 T 확정 전까지 두 모드는 같은 경로·같은 속도로 달린다(접근 속도 제한은 0.25 m/s에서 T 8 cm 안에서만 걸리고, rule도 14 m 전에는 모든 T를 거부). rule 자신도 반복 사이에 15 → 12로 흔들렸으므로 잡음으로 본다. 평균 진행률 −1.3 %p도 같은 이유(rule 반복 사이 차이 6.5 %p).
- 짧은 맵(5a, `short_t` 8 m)은 두 모드 모두 4회 모두 3.1 m 첫 코너에서 이탈해 Gazebo로는 확인하지 못했다. 주행 거리 게이트가 없다는 것은 단위 시험(주행 0 m에서 실제 T 수락)으로만 확인했다.
- 튜닝 대처표(HILL·ARC 속도 0.18)는 튜닝 맵에서는 나았지만 보류 세트에서 4/29로 빈 표(10/29, 9/29)보다 나빴다 → **채택하지 않음**(`config/playbook_tuned.yaml`로만 보관).

### 8.3 채택 결정

- **채택(권고)**: `controller: playbook` + 빈 대처표 = **학습 분류기 종점 판정**. 같은 시험에서 rule보다 완주가 많고 잘못된 완주가 없으며, 진행률·랩은 잡음 범위 안에서 같다. 설정 파일은 `config/playbook_rule.yaml`(`mode:=playbook_rule`).
- **기본값은 바꾸지 않았다**: launch의 기본 `mode:=rule`은 launch 담당(다른 에이전트) 파일이고 `rule.yaml`은 룰 담당 파일이다. 기본으로 올리려면 launch 기본값을 `playbook_rule`로 바꾸면 된다(팀 결정 필요).
- **채택하지 않음**: 상황별 속도 대처표(§7의 튜닝값). 남은 실패의 대부분이 첫 원호에서 지도가 선을 잃는 인지 문제라 대처표로 풀리지 않는다.


## 9. 재현

```bash
LAB_NAME=lf_l tools/lab_env.sh up      # 컨테이너 안 /ws 에서, ROS 환경 source 후
python3 experiments/situation/collect.py maps
python3 experiments/situation/collect.py gazebo gz_train && python3 experiments/situation/collect.py gazebo gz_val
python3 experiments/situation/collect.py synth --workers 4
python3 experiments/situation/collect.py synth --workers 2 --variants markings --out synth_markings
python3 experiments/situation/collect.py build
python3 experiments/situation/train.py                     # -> config/situation_model.yaml, test/data/situation_fixture.npz
python3 experiments/situation/tune.py step --name ... --set SITUATION.param=v1,v2
python3 experiments/situation/evaluate.py run --tag rule --params src/line_follower/config/rule.yaml --groups official,seeds,random,finishgate
python3 experiments/situation/evaluate.py run --tag playbook --params src/line_follower/config/playbook_rule.yaml --groups official,seeds,random,finishgate
python3 experiments/situation/evaluate.py compare rule playbook
```

## 10. 남은 일

- **`follower_node.py` 훅 제안(이 작업에서는 수정하지 않음, 담당 에이전트에게 전달)**:
  1. 경로가 없을 때(0.35 m 직진 후 회전)도 컨트롤러에 묻는 훅(예: `Controller.recover(dt)`가 None이면 기존 동작). 남은 실패의 대부분이 원호 안에서 경로를 잃은 뒤 이 직진으로 선을 벗어나는 경우라, 상황(ARC/RIGHT_ANGLE)을 알면 직진 대신 그 방향으로 돌 수 있다.
  2. 확정한 `t_point`를 되돌리는 훅(확정 뒤 막대 너머에 선이 보이면 해제, safety 권고 4). 지금은 한 번 확정하면 되돌릴 수 없어 판정을 늦추는 쪽(3프레임 + 시야 조건)으로만 막았다.
  3. 디버그 영상에 현재 상황·P(FINISH_T) 표시.
- START_T는 Gazebo 표본이 0개다(합성 10 152개로만 학습, 단위 시험으로만 확인). 출발 T로 되돌아오는 실제 주행 수집이 필요하다.
- 약한 클래스: STUB(재현율 0.55, 표본 95), GAP(0.30), STRAIGHT(0.53). 곁가지·끊긴 길 근처 출발 구간을 늘린 수집, GAP 라벨 정의(노드가 7 cm까지는 메워서 실제로는 안 보임) 재검토.
- 튜닝은 맵 12개·설정당 12–24회라 회차 잡음(J ±150)에 비해 작다. 대처표를 다시 시도한다면 인지 쪽 개선(원호 지도 상실) 뒤에, 맵·반복을 늘려서 한다. `omega_max`·`lookahead_*`·CROSSING/LOOP_CORNER 행은 시도하지 않았다.
- `control/playbook.py`가 PyYAML을 쓴다. ROS 2 Jazzy에는 기본으로 있지만 `package.xml`에 `python3-yaml` exec_depend를 넣는 것이 맞다(이 작업의 담당 파일이 아니라 넣지 않음).
- 평가 세트를 늘리기: official_like50·hills60의 보류 시드, 짧은 맵(5a)에서 첫 코너를 넘는 경우.

