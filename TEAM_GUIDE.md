# 팀 작업 안내

> 3인 분업(인지 / 판단 / 제어)과 각 파트의 고정·개선 대상은 [`docs/TEAM_ROLES.md`](docs/TEAM_ROLES.md)를 먼저 본다.

브랜치 `wlsgur0317/two-track` 기준이다. 제출용 ROS 패키지는 `src/line_follower` 하나다. 인식과 제어를 플러그인으로 나눠서 **룰 기반 담당과 딥러닝 담당이 서로 다른 파일만 고치도록** 만들었다.

## 1. 구조

```
src/line_follower/
  line_follower/
    follower_node.py      # [공용] ROS 노드: 토픽, 오도메트리+IMU 자세, 선 지도 갱신, 경로 추적, 종점 판정
    line_map.py           # [공용] 카메라로 본 선을 odom 격자 지도(5 mm)에 기록하고 추적한다
    vision.py             # [공용] 카메라 → 지면(BEV) 투영. bev_warp()는 모든 인식 플러그인이 같이 쓴다
    perception/           # 인식 플러그인: 카메라 영상 → BEV 마스크
      base.py             #   [공용] 인터페이스 PerceptionOutput(line, valid, obstacle)
      rule.py             #   [룰 기반] OpenCV 임계값 (사용자 원본 알고리즘)
      dl.py               #   [딥러닝] 분할 모델 (빈 틀, TODO 표시)
    control/              # 제어 플러그인: 앞쪽 경로 → (v, w)
      base.py             #   [공용] 인터페이스 compute(rp, dt) -> (v, w)
      rule.py             #   [룰 기반] pure pursuit + 곡률 속도 제한 (사용자 원본, 동작 그대로)
      learned.py          #   [딥러닝] 학습 정책. 룰 제어에 보정값을 더하는 잔차 방식 (빈 틀)
  config/
    rule.yaml             # perception: rule, controller: rule
    dl.yaml               # perception: dl,   controller: rule (또는 learned)
    follower.yaml         # 원본 설정(= rule)
  launch/line_follower.launch.py
```

선 지도 방식은 모든 모드의 공통 기반이다. 카메라는 로봇 앞 0.31 m 안을 못 보므로, 본 선을 odom 지도에 기록해 두고 로봇 바로 앞과 아래의 선은 지도로 따라간다.

## 2. 담당

| 담당 | 고치는 파일 | 고치지 않는 파일 |
|---|---|---|
| **룰 기반** | `perception/rule.py`, `control/rule.py`, `config/rule.yaml` | `perception/dl.py`, `control/learned.py`, `config/dl.yaml` |
| **딥러닝** | `perception/dl.py`, `control/learned.py`, `config/dl.yaml`, `experiments/dl/`, `experiments/rl/` | `perception/rule.py`, `control/rule.py`, `config/rule.yaml` |
| **공용** (바꿀 때 서로 알릴 것) | `follower_node.py`, `line_map.py`, `vision.py`, `*/base.py`, `launch/`, `experiments/bench.py`, `experiments/telemetry.py` | — |

플러그인 인터페이스(`base.py`)를 바꾸면 양쪽이 모두 깨진다. 꼭 바꿔야 할 때만 합의해서 바꾼다.

## 3. 실행

```bash
ros2 launch line_follower line_follower.launch.py                 # 룰 기반 (기본)
ros2 launch line_follower line_follower.launch.py mode:=dl        # 딥러닝 (dl.yaml의 dl.model_path 필요)
ros2 launch line_follower line_follower.launch.py gui:=false      # 헤드리스
```
노드는 launch 20초 뒤에 시작한다(과제 규칙). 플러그인은 파라미터 `perception`(rule|dl), `controller`(rule|learned)로도 고를 수 있다. 딥러닝 플러그인은 모델 경로가 없으면 **조용히 룰로 바뀌지 않고 바로 오류를 낸다.**

## 4. 채점과 기록 (개발 도구, 로봇 런타임에서는 쓰지 않는다)

Docker 랩 컨테이너 안에서 돌린다(`LAB_NAME=<이름> tools/lab_env.sh build`).

```bash
# 공식 트랙 3회 (bench가 Gazebo, 브리지, 노드를 띄우고 지상 진실로 채점)
python3 experiments/bench.py src/line_follower_bringup/worlds/line_track.sdf \
  src/line_follower_bringup/worlds/line_track.sdf src/line_follower_bringup/worlds/line_track.sdf \
  --follower experiments/user_base/run_user.py --jobs 3 --off 0.12 --output out/official
# 랜덤 맵
python3 experiments/worlds/gen_world.py --seed 5 --difficulty normal --obstacles 0
# 여러 주행 겹쳐 보기
python3 experiments/telemetry.py compare out.png out/official
# 선 지도 vs 실제 선 (run_user_dump.py로 돌린 경우)
python3 experiments/user_base/compare_map.py map.npz out/official/01_01 map.png
```
- 모드 바꾸기: `LF_PARAMS=src/line_follower/config/dl.yaml`를 주고 `run_user.py`를 돌린다.
- 모든 주행 폴더에는 `telemetry.{csv,npz,json,png}`가 남는다. x축은 선을 따라 간 거리이고, 횡오차, 속도, 가속도, 바퀴 선 접촉, 최대 도달 거리가 들어 있다. 자세한 내용은 `experiments/README_telemetry.md`.
- **ROS 도메인 충돌 주의:** 랩 컨테이너끼리 같은 도메인이 서로 보인다. bench는 `LAB_NAME`별로 도메인 블록을 쓴다(lf_a 30–49, lf_b 50–69, lf_c 70–89). 팀원마다 다른 `LAB_NAME`을 쓰고, 노드용 `ROS_DOMAIN_ID`도 서로 다르게 잡는다.
- 판정 기준: 이탈은 `--off 0.12`(선이 바퀴 사이에 있으면 된다. 바퀴 안쪽 가장자리 ±0.1345 m)를 쓴다. 완주율은 같은 설정 10회 중 몇 회인지로 본다.

## 4-1. 테스트 맵 세트 (대량 평가)

평가 맵은 공식 트랙과 다르고 더 어렵다. 그래서 다양한 요소를 섞은 랜덤 맵을 대량으로 만들어 쓴다. 생성기: `experiments/worlds/gen_world.py`.

| 요소 | 옵션 | 설명 |
|---|---|---|
| 난이도 | `--difficulty easy/normal/hard` | 원호 코너 반지름(hard 8–17 cm), 직각 코너 수, 길을 가로지르는 큰 보조 루프 |
| 언덕 | `--hills N` | 언덕 여러 개(첫 직선 외 다른 직선에도) |
| 좌우 반전 | `--mirror` | 코너 방향이 반대, 출발 방향 −x |
| 코너 원 | `--loop-corners N` | 공식 트랙의 작은 원. 직각 코너 대신 반대 방향으로 270° 돌며 지나온 길을 90°로 가로지른다(r 8–12 cm) |
| 연결 교차로 | `--detours N` | 직선에서 빠져나가 돈 뒤, 이미 지나온 직선을 90°로 가로질러 내려가고 원래 길로 복귀한다 |
| 교차 막대 | `--crosses N` | 길을 직각으로 가로지르는 +자 선. 직진해서 통과한다 |
| 곁가지 | `--stubs N` | 한쪽으로만 뻗은 짧은 가지(길 중간 T자). 종점이 아니다 |
| 끊긴 길 | `--gaps N` | 바닥 선에 5–15 cm 빈 구간. 채점 중심선은 이어져 있다 |
| 물방울 원 | `--small-loops N` | 직선 위 자기 교차 루프. 진입과 진출이 일직선이라 **직진도 정답일 수 있는 애매한 경우**라 스트레스 세트에만 쓴다 |
| 상자 | `--obstacles N` | 장애물 |

모든 추가 요소의 기본값은 0이다. 옵션을 안 주면 기존 시드 맵(평가 세트 0–39)이 바이트 단위로 똑같이 나온다.

미리 만든 세트(`experiments/worlds/suites/<이름>/`, 맵 파일은 git에 없으니 아래 명령으로 다시 만든다. 결정적이라 같은 결과가 나온다):

| 세트 | 맵 수 | 시드 | 내용 |
|---|---|---|---|
| `general200` | 200 | 1000– | 모든 요소를 무작위로 섞음, 장애물 없음 |
| `obstacles100` | 100 | 3000– | 위와 같고 상자 1–3개 |
| `official_like50` | 50 | 5000– | hard/normal, 언덕 2–4, 코너 원 1–2, 연결 교차로 1, 곁가지, 끊긴 길 |
| `stress50` | 50 | 7000– | 모든 요소 최대, 물방울 원 포함 |

```bash
python3 experiments/worlds/make_suite.py --name general200 --count 200
python3 experiments/worlds/make_suite.py --name obstacles100 --count 100 --seed0 3000 --obstacles 1-3
python3 experiments/worlds/make_suite.py --name official_like50 --count 50 --seed0 5000 --difficulty hard,normal \
  --hills 2-4 --loop-corners 1-2 --detours 1 --stubs 1-2 --crosses 0-2 --gaps 1-3
python3 experiments/worlds/make_suite.py --name stress50 --count 50 --seed0 7000 --difficulty hard --hills 3-4 \
  --loop-corners 2 --detours 1 --stubs 2 --crosses 2 --gaps 3 --small-loops 1-2
# 세트 전체 주행 (컨테이너 안)
python3 experiments/bench.py $(cat experiments/worlds/suites/general200/list.txt) \
  --follower experiments/user_base/run_user.py --jobs 4 --off 0.12 --output out/general200
# 요소별 완주율 표 + 그림
python3 experiments/worlds/summarize_suite.py out/general200 experiments/worlds/suites/general200/index.csv --png general200.png
```
각 세트 폴더에는 `list.txt`(bench 입력), `index.csv`(맵별 요소), `sheet_XX.png`(미리보기 20장씩)가 있다. 시드 1000 이상은 튜닝용이고, 0–39는 고정 평가 세트라 튜닝에 쓰지 않는다.

## 5. 과제 규칙 (공통)

- 로봇 런타임은 `/camera/image_raw`, `/imu`, `/odom`, `/cmd_vel`만 쓴다. `/world/...` 토픽, `set_pose`, 지상 진실은 채점 도구에서만 쓴다.
- `worlds/`, `models/` 원본은 수정하지 않는다(SHA256 동일).
- 실행은 `ros2 launch` 한 줄이고, 20초 뒤에 출발한다.

## 6. 현재 기준선 (2026-11-24, 리드 측정)

| 설정 | 공식 트랙 | 장애물 없는 랜덤 20맵 |
|---|---|---|
| `rule` (사용자 원본 = 이 틀의 기본값) | 완주 63–64초. 3회 중 2회 완주(1회는 9.55 m 이탈) | 5/20 완주, 평균 47.9%, 완주 평균 68초 |
| 이전 실험 follower (`experiments/follower.py`, 참고용) | 10회 중 9회, 약 177초 | 8/20, 67.1%, 완주 평균 223초 |

공식 트랙에서 빠른 이유의 일부는 `speed_profile`(주행 거리 구간별 속도)이 **공식 트랙에 맞춘 값**이기 때문이다. 평가 맵은 공식 트랙과 다르니, 룰 담당의 1순위는 이 의존을 없애는 것이다.

## 7. 알려진 문제와 다음 할 일

**룰 기반**
1. 랜덤 맵 실패 14건 중 10건이 2.7–3.7 m의 첫 급코너에서 났다.
2. **코너마다 지도 기준점이 밀린다.** 급코너를 돌 때마다 지도와 자세 추정이 진행하던 방향으로 13–25 cm 밀린다(제자리 회전 때 바퀴 슬립). 직선은 1–3 cm로 정확하다. 그림은 `experiments/results/lead/linemap/`에 있다. 해결 후보는 두 가지다. 하나는 코너를 지난 직후 카메라로 본 선을 기존 지도에 정합해 자세를 보정하는 것이고, 다른 하나는 급코너 직전에 감속해 슬립을 줄이는 것이다.
3. 출발 직후 실제 선이 없는 곳에 쐐기 모양 가짜 선 덩어리가 기록된다. 화면 전체가 어둡다고 판정되는 순간으로 보인다(임계값 0.62×median).
4. `speed_profile`과 `finish_min_dist`(14 m) 같은 공식 트랙 전용 설정을 일반화한다.

**딥러닝**
1. `perception/dl.py`의 TODO 두 개(`load`, `segment`)를 채운다. 학습된 모델과 코드는 `experiments/dl/`에 있다. 라인은 `line_seg_v2.onnx`(공식 트랙 라인 IoU 0.942)를 쓰고, 장애물은 v1이 낫다. 이 둘을 `vision.bev_warp`로 BEV에 맞추면 된다.
2. 장애물: `node.last_obstacle`에 BEV 장애물 마스크가 들어온다. 이걸 경로 계획(비켜 지나가고 복귀)에 연결하는 게 다음 단계다. G2 장애물 평가 세트는 `experiments/worlds/eval_set_G2.txt`이다.
3. 강화학습(선택): `control/learned.py`. 선 지도 위 잔차 정책이고 2D 시뮬레이터는 `experiments/rl/`에 있다. 1차 PPO 파일럿은 학습량이 부족해 기존 제어보다 못했다.

## 8. 브랜치와 병합

- 기준 브랜치: `wlsgur0317/two-track`
- 작업 브랜치: 룰 담당은 `rule/<이름>`, 딥러닝 담당은 `dl/<이름>`. 기준 브랜치에서 따서 작업한다.
- 병합 조건: 공식 트랙 회귀가 없고(같은 설정 3회 이상), 자기 담당 지표가 기준선보다 좋아야 한다. 결과 폴더와 `telemetry.json` 수치를 PR 본문에 적는다.

## 4-2. 목적별 케이스 세트 (`suites/cases`, 334개, 장애물 없음)

`python3 experiments/worlds/gen_cases.py --preview`로 만든다. 트랙마다 상황이 하나씩 들어간다. 모든 케이스는 좌우 반전 쌍으로 만든다. 교차 지점의 위치와 각도는 생성 후 자동으로 계산해 JSON `case.crossings`에 넣는다.

| 분류 | 그룹 (개수) |
|---|---|
| 기하 | corner90 반경 0–0.40 m (14), turn_angle 30–150° (28), s_curve (24), hairpin (14), zigzag (12), wavy (8), spiral (2) |
| 교차·겹침 | cross_bar 30–90° (10), multi_cross (6), corner_loop (8), self_cross 30–90° (14), parallel 3–20 cm·곡선 (26), y_split (16), y_merge (16), t_stub (18), overlap (4), touching (6), near_track (2) |
| 선 품질 | gap 3–20 cm 직선·곡선 (20), dashed (6) |
| 언덕 | ramp (6), ramp_then_turn 높이×거리×코너 종류 (48), turn_then_ramp (8), ramp_cross (4), ramp_gap (4), double_ramp (4) |
| 출발·도착 | finish_like (6) |

- 그룹별 목록은 `list_<그룹>.txt`, 미리보기는 `preview_<그룹>.png`다.
- 결과 집계는 `python3 experiments/worlds/summarize_cases.py <bench 출력> [--by-name]`로 한다. 실패 위치를 케이스 구간 앞, 구간 안, 구간 뒤로 나눠 센다.
