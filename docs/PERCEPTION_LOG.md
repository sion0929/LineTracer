# 인지 로그 (판단 팀용)

2026-11-27 · 인지 파트

판단이 인지 출력의 잡음을 어떻게 거를지 설계할 수 있도록, 인지가 **매 프레임 판단에 넘기는 것**을 그대로 기록한 로그다. 판단은 지금 인지(V0) 기준으로 만든다(`AGENTS.md` R5). 인지는 여기서 나빠지지 않으므로, 이 로그의 잡음을 견디는 판단이면 인지가 좋아져도 손해가 없다.

## 1. 받은 로그 100개

| 항목 | 값 |
|---|---|
| 위치 | `experiments/planning_logs/V0-NearRule640-width5-logodds/<번호>_decoy_<맵>/` |
| 맵 | decoy1000 학습 맵 0000–0099 (언덕 있는 맵 85개, 가짜 길 평균 1.6개). 정답 기하는 `bash experiments/worlds/make_decoy1000.sh 0`으로 만든다 |
| 주행 | Isaac Sim, **정답 경로를 0.2 m/s로 따라감**(재생). 판단·제어는 쓰지 않았다. 그래서 로그는 판단과 무관한 순수한 인지 출력이다 |
| 인지 | V0-NearRule640 + 폭 필터 5 cm + log-odds 지도 (`experiments/isaac/params/V0-NearRule640-width5-logodds.yaml`) |
| 렌더링 | 무작위 조명·바닥 텍스처 (시드 = 맵 번호). 그림자 오검출이 들어 있다 |
| 크기 | 맵당 약 0.3 MB, 2000프레임 내외(30 Hz, 약 66 s) |

> 폐루프 기본값은 V0(폭 필터·log-odds 없음)다. 공식 트랙 10회에서 V0가 10/10으로 가장 안정적이었다. 폭 필터를 빼면 무작위 조명에서 오검출이 더 많다(새 맵 10개 오프라인 정밀도 V0 0.909, 폭 필터 0.996). 판단은 **이 로그보다 오검출이 더 많을 수 있다**고 보고 설계한다.

각 폴더:

| 파일 | 내용 |
|---|---|
| `perception_log/` | 인지 로그 (아래 2장) |
| `trajectory.npy` | 정답 주행 궤적, 열 `[sim_s, x, y, s, lateral_error]` (월드 좌표, m) |
| `job.json` | 맵 경로(저장소 기준), `odom_origin` = odom 원점의 월드 자세 `[x, y, yaw]` |
| `result.json` | 주행 결과 요약 |

## 2. 형식 (`lf-perception-log/2`, 받은 100개는 `/1`)

`perception_log/meta.json`과 `chunk_NNNN.npz`(60프레임씩)로 되어 있다. 프레임 하나 = 지도에 들어간 카메라 프레임 하나.

| 필드 | 모양 | 뜻 |
|---|---|---|
| `t` | (N,) | 촬영 시각 [s] |
| `pose` | (N, 3) | 촬영 순간의 자세 `(x, y, yaw)`, **odom** [m, rad] |
| `pitch` | (N,) | 투영에 쓴 카메라 피치 [rad] (언덕에서 바뀐다) |
| `travel` | (N,) | 누적 주행 거리 [m] |
| `used` | (N,) | 이 프레임으로 지도를 고쳤는가 |
| `line` | (N, BH, BW) | BEV 선 마스크 (bool) |
| `valid` | (N, BH, BW) | BEV에서 카메라가 실제로 본 칸 |
| `far` | (N, BH, BW) | 먼 띠로 본 칸 (V0는 전부 0) |
| `imu_pitch` | (N,) | 걸러 낸 IMU 몸체 피치 [rad], 앞이 들리면 + (형식 2부터) |
| `elev` | (N,) | 출발 바닥 기준 바퀴 높이 [m], 오도메트리 + IMU로 적분 (형식 2부터) |
| `slope` | (N,) | 언덕 상태 0 평지, 1 오르막, 2 정상 평지, 3 내리막 (형식 2부터) |

형식 1 로그(받은 100개)에는 마지막 세 필드가 없다. 로더는 이때 `imu_pitch`, `elev`를 NaN으로, `slope`를 −1로 채운다. 형식 1의 `pitch`는 **투영에 쓴 카메라 피치**라서 경사면 위에서는 평지 값(0.33°)으로 나온다. 언덕 구간은 `pose`와 맵 JSON의 `ramps`로 찾는다.

BEV 격자는 `meta.json`의 `grid`에 있다: 칸 5 mm, 차축 앞 `x_near`–`x_far`(0.31–0.85 m), 좌우 ±`y_half`(0.55 m). **행 0이 가장 먼 곳**, 마지막 행이 가장 가까운 곳이고, **열 0이 왼쪽(+y)**이다. 칸 `(r, c)`의 중심은

```
x = x_far - (r + 0.5) * res      # 앞
y = y_half - (c + 0.5) * res     # 왼쪽
odom 점 = pose_xy + R(yaw) @ (x, y)
```

## 3. 읽기

```python
from line_follower.perception.log import load      # ROS 패키지 쪽 로더
meta, d = load('experiments/planning_logs/V0-NearRule640-width5-logodds/0001_decoy_0000/perception_log')
d['line'].shape   # (N, BH, BW) bool
d['pose'][i]      # (x, y, yaw) in odom
```

ROS 없이 쓰는 도구 (`numpy`, `opencv` 필요):

```bash
T=experiments/eval/perception_log_tool.py
L=experiments/planning_logs/V0-NearRule640-width5-logodds
python3 $T summary $L/*/perception_log                    # 맵별 프레임 수, 선 칸 수, 선 없는 프레임 비율, 피치 범위
python3 $T rebuild $L/0001_decoy_0000/perception_log --world --out map.png   # 로그로 지도를 다시 쌓아 정답 중심선(빨강)과 겹쳐 그림
python3 $T frames  $L/0001_decoy_0000/perception_log --out frames/ --every 10  # 프레임별 BEV 마스크 PNG
```

`--world`는 `job.json`의 맵 JSON이 있어야 정답 선을 그린다. 먼저 `bash experiments/worlds/make_decoy1000.sh 0`을 돌린다.

## 4. 판단이 이 로그로 할 수 있는 것

- 판단 코드에 프레임을 순서대로 넣어 **주행 없이** 길 선택을 재현한다. 로그의 `line`/`valid`를 `pose`로 `LineMap.update()`에 넣으면 주행 때와 같은 지도가 된다.
- 오검출 거르기 규칙을 시험한다. 예: 짧고 고립된 조각, 경로에서 먼 덩어리, 한 번만 본 먼 선, 그림자처럼 두꺼운 덩어리.
- 가짜 길(막다른 교차, 곁가지) 고르기를 정답(`trajectory.npy`, 맵 JSON의 `centerline`)과 비교해 채점한다.
- 언덕에서 피치가 바뀔 때(`pitch`) 선이 어떻게 흔들리는지 본다.

## 5. 새 로그 만들기

Isaac 주행은 로그를 기본으로 남긴다(`ISAAC_PERCEPTION_LOG=0`이면 끔). 출력 폴더 안 `perception_log/`에 생긴다. 실물이나 Gazebo에서는 노드 파라미터 `perception_log:=<폴더>` 또는 환경 변수 `LF_PERCEPTION_LOG=<폴더>`로 켠다. 기록 실패는 주행에 영향을 주지 않는다(로그만 꺼진다).

같은 100개를 다시 만드는 배치: `experiments/isaac/run_replay_eval.sh <맵 목록> <출력> experiments/isaac/params/V0-NearRule640-width5-logodds.yaml` (`ISAAC_RENDER_ARGS="--lights --texture"`, 맵당 약 3분). 끝난 배치를 이 폴더 형식으로 옮기는 것은 `python3 experiments/isaac/export_planning_logs.py <출력>/runs experiments/planning_logs/<설정 이름>`.
