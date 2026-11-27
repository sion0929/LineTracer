# 실물 로봇 테스트 체크리스트

2026-11-26 · TurtleBot3 Waffle Pi / Jetson Orin Nano

목적은 두 가지다. 하나는 시뮬레이션과 같은 조건에서 주행 데이터를 남기는 것, 다른 하나는 정지·직진 기록으로 IMU·엔코더·카메라의 실제 잡음 크기를 재는 것이다. 결과는 `experiments/real/analyze_static.py`로 분석한다. 이 문서는 그대로 따라 할 수 있는 순서만 담는다.

관련 파일:

| 파일 | 역할 |
|---|---|
| `src/line_follower/line_follower/vision.py` | 카메라 모델 (640×480, HFOV 1.085595 rad, 축에서 0.076 m 앞, 지면에서 0.103 m 위) |
| `src/line_follower/config/rule.yaml` | 시뮬레이션 설정. `use_sim_time: true`이므로 실물에서 그대로 쓰지 않는다 |
| `experiments/jetson/robot.launch.py` | 실물용 launch (`use_sim_time:=false`) |
| `experiments/real/record_run.sh` | rosbag + 메타데이터 YAML 기록 |
| `experiments/real/analyze_static.py` | 정지·직진 bag 분석 |

---

## 0. 준비물

- 줄자(3 m 이상), 마스킹 테이프, 직각자 또는 삼각자
- 로봇 배터리(충전 완료), 예비 배터리, 전압계
- 노트북(ROS 2 Jazzy, 같은 `ROS_DOMAIN_ID`), 카메라/조명 사진용 휴대폰
- 정지 수단(비상정지 버튼 또는 텔레옵 조종기, 그리고 아래의 `/cmd_vel` 0 명령)
- 인쇄한 이 체크리스트와 `run_log.md`

---

## 1. 시작 전

### 1.1 코드와 실행 환경

- [ ] 로봇과 노트북의 `ROS_DOMAIN_ID`가 같다.
- [ ] `ros2 topic list`에 `/camera/image_raw`, `/odom`, `/imu`, `/cmd_vel`이 보인다.
- [ ] `ros2 topic hz /camera/image_raw`가 약 30 Hz, `/imu`가 100 Hz 이상, `/odom`이 20 Hz 이상인지 확인한다.
- [ ] `ros2 topic echo /camera/image_raw --field width --once`가 640, `height`가 480인지 확인한다.
- [ ] 실물에서는 `robot.launch.py`를 쓴다. `config/rule.yaml`은 시뮬레이션용(`use_sim_time: true`, 트랙별 `speed_profile`)이므로 그대로 load하지 않는다.
- [ ] 첫 주행은 `v_max`를 0.10–0.15 m/s로 낮춘다. `ros2 param set /follower_node v_max 0.12`처럼 실행 중에 바꾸거나, 실물용 YAML 사본을 만들어 `speed_profile: [0.0]`로 둔다.
- [ ] 기록 스크립트는 ROS 2가 source된 셸에서 실행한다. 컨테이너와 호스트가 같은 네트워크/도메인이면 어느 쪽에서 실행해도 된다.

### 1.2 카메라 장착 확인

먼저 로봇을 평평한 바닥에 두고 모터를 끈다. 축(바퀴 축) 위치를 바닥에 표시하고, 렌즈 중심까지의 값을 잰다.

- [ ] 높이: 바닥에서 렌즈 중심까지 **0.103 m** (허용 ±0.005 m). 다르면 실제 값을 적어 두고 리드에게 알린다. `vision.py`의 `CAM_H`를 임의로 고치지 않는다.
- [ ] 전후 위치: 축에서 렌즈 중심까지 **0.076 m 앞**. 줄자를 축 표시에서 렌즈 중심까지 수평으로 잰다.
- [ ] 좌우 수평(roll): 카메라가 기울지 않았는지 눈으로 확인한다. 아래 테이프 방법에서 선이 좌우로 기울면 roll이 있다.
- [ ] 렌즈/브래킷 나사가 풀리지 않았는지, 케이블이 렌즈를 당기지 않는지 확인한다.

#### 테이프로 피치 재기

`vision.py`는 카메라 광축이 지면과 수평인 모델(θ=0)이다. 노드의 `REST_PITCH`는 0.33°(앞이 살짝 들린 상태)를 가정한다. 두 값의 차이는 0.6 m에서 약 3 px이므로, 어느 쪽에 맞출지 정해 두고 기록한다. 실물 기본값은 `REST_PITCH`에 맞추는 것을 권한다.

1. 축 위치에서 앞으로 0.4, 0.6, 0.8, 1.0 m 지점에 바닥과 수직인 테이프 선을 붙인다. 선은 로봇 진행축과 직각이어야 한다(양쪽 같은 거리 확인).
2. 로봇을 축 표시 위에 두고, 진행 방향은 테이프와 수직으로 맞춘다. 로봇은 움직이지 않는다.
3. `rqt_image_view`로 `/camera/image_raw`를 열고 한 장을 저장한다. 파일 이름은 `cam_check_<날짜>.png`.
4. 그림에서 각 테이프 선의 **행(row, 위가 0)** 을 읽는다. 이미지 뷰어가 좌표를 보여 주지 않으면, 예상 행 근처만 잘라 픽셀 좌표를 읽어도 된다.
5. 아래 표와 비교한다. 행이 표보다 크면 카메라가 위로 들린 것(+), 작으면 아래로 숙인 것이다. **5 px ≈ 0.5°**, 약 **10 px ≈ 1°**다.

| 축에서 거리 x | θ=0° 예상 행 | θ=0.33° 예상 행 |
|---:|---:|---:|
| 0.4 m | 408 | 412 |
| 0.6 m | 344 | 347 |
| 0.8 m | 315 | 318 |
| 1.0 m | 299 | 302 |

정확한 피치는 다음 식으로 구한다. `v`는 읽은 행, `CY=239.5`, `FX=530.47`, `CAM_H=0.103`, `xc = x - 0.076`이다.

```text
tau = (v - CY) / FX
theta = atan2(tau*xc - CAM_H, tau*CAM_H + xc)   # rad, +면 위로 들림
```

- [ ] 0.6 m에서 읽은 행이 목표(0° 또는 0.33°)에서 ±5 px 안쪽이다.
- [ ] 1.0 m에서도 같은 방향으로 ±10 px 안쪽이다. 거리에 따라 오차가 커지면 높이(`CAM_H`)가 모델과 다른 것이다.
- [ ] 테이프 선이 좌우로 기울지 않는다. 왼쪽과 오른쪽 끝의 행 차이가 3 px를 넘으면 roll을 의심하고 브래킷을 다시 맞춘다.
- [ ] 높이나 피치가 허용 범위를 벗어나면 억지로 구부리지 말고, 실제 값을 `run_log.md`와 메타데이터 `notes`에 적는다.

### 1.3 노출과 화이트밸런스

`vision.py`의 선 검출은 "화면 아래 지면 밝기의 중앙값 × 0.62"를 문턱으로 쓴다. 전체 밝기가 조금 달라도 적응하지만, 포화(하얗게 날아감)와 심한 플리커에는 약하다.

- [ ] 카메라 제어값을 저장한다: `v4l2-ctl --list-ctrls > cam_controls_<날짜>.txt` (UVC 카메라인 경우).
- [ ] 같은 세션에서는 노출과 화이트밸런스를 **고정**한다. 자동 노출/자동 WB가 계속 바뀌면 run끼리 비교할 수 없다.
  - UVC 예: `v4l2-ctl -c auto_exposure=1 -c exposure_time_absolute=<값>`
  - WB 예: `v4l2-ctl -c white_balance_automatic=0 -c white_balance_temperature=<값>`
  - 드라이버마다 제어 이름이 다르므로 `v4l2-ctl --list-ctrls`에 있는 이름을 쓴다.
- [ ] 매트가 하얗게 포화되지 않았는지, 선이 충분히 어두운지 확인한다. 대략 매트 90–200, 선 60 이하를 목표로 한다.
- [ ] 50/60 Hz 조명 아래에서 가로 줄무늬(밴딩)가 보이면 노출 시간을 10 ms(50 Hz) 또는 8.33 ms(60 Hz)의 배수로 맞춘다.
- [ ] 조명이 바뀌면(햇빛 방향, 형광등 교체 등) 다시 고정하고 그 사실을 적는다.

### 1.4 조명 사진

- [ ] 로봇 높이에서 진행 방향, 좌우, 위쪽(천장)을 찍는다. 시작점·중간·먼 쪽에서 각각 찍는다.
- [ ] 창문, 직사광, 그림자 띠, 바닥 반사, 조명 종류(형광등/LED/백열)를 메모한다.
- [ ] 파일은 `lighting/<날짜>_<장소>_<방향>.jpg`로 저장한다.
- [ ] run 도중 조명이 바뀌면 사진을 다시 찍고 `run_log.md`에 시각을 적는다.

### 1.5 시작 위치 T와 스폰

- [ ] 로봇의 바퀴 축을 출발/도착선 T 위에 정확히 둔다. 로봇 중심이 선의 가운데에 오게 한다.
- [ ] 진행 방향은 선이 이어지는 쪽으로 맞춘다. 선이 카메라 앞 0.31–0.62 m 구간에 보여야 한다.
- [ ] 주행 경로 앞 0.5 m 안에 손, 케이블, 물체가 없다.
- [ ] `finish_min_dist`를 확인한다. 코드 기본값은 10 m, `rule.yaml`은 14 m다. 한 바퀴 트랙 길이보다 작아야 T에서 멈춘다. 실물 설정에 이 값을 적는다.
- [ ] `ros2 topic echo /odom --once`와 `ros2 topic echo /imu --once`가 들어오는지, `/cmd_vel`이 0인지 확인한 뒤 시작한다.

### 1.6 배터리와 안전

- [ ] 배터리를 만충한다. 전압과 충전 상태를 적는다.
- [ ] 저전압이면 속도와 엔코더 값이 달라지므로 주행을 중단한다.
- [ ] 비상정지 버튼 또는 텔레옵 정지 버튼을 손 닿는 곳에 둔다.
- [ ] 로봇을 정지시키는 명령을 미리 익힌다:
  ```bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
  ```
- [ ] 첫 동작은 바퀴를 블록 위에 올린 채로 `/cmd_vel`과 바퀴 방향을 확인한다.
- [ ] 첫 주행은 낮은 속도로 한다. 사람이 로봇 옆에서 언제든 정지할 수 있게 한다.
- [ ] 로봇이 선을 벗어나거나, 센서 rate가 떨어지거나, 이상한 소리가 나면 즉시 멈추고 원인을 적는다.

---

## 2. 주행 중

모든 run의 공통 순서:

1. `record_run.sh`를 먼저 실행하고 1–2초 기다린다.
2. 로봇을 움직인다. 끝나면 1–2초 더 세워 둔 뒤 기록을 멈춘다.
3. `ros2 topic hz`로 카메라·IMU·odom rate를 확인한다. 카메라가 20 Hz 아래로 떨어지거나 큰 gap이 보이면 그 run은 버린다.
4. 실제 이동 거리, 속도, 결과, 특이사항을 `run_log.md`에 적는다.

기록 명령 예:

```bash
cd ~/26_robotprogramming_project
OUT_DIR=$PWD/bags/real/20261126_tb3 \
  ./experiments/real/record_run.sh static_30s --notes "정지, 모터 off, 30초"
```

스크립트는 `/camera/image_raw`가 없고 `/camera/image_raw/compressed`가 있으면 압축 토픽을 기록한다. 단, follower는 raw 토픽을 구독하므로 follower 주행에서는 raw가 반드시 있어야 한다.

bag에는 카메라(raw 또는 compressed), `/camera/camera_info`, `/odom`, `/imu`, `/cmd_vel`, `/tf`, `/tf_static`, `/line_follower/debug`가 들어간다. 실행 중에 없는 토픽은 건너뛴다.

### 2.1 정지 30초 (IMU·odom·카메라 기준)

- [ ] follower를 끄거나 `v_max`를 0으로 둔다.
- [ ] 로봇을 평평한 바닥에 두고 30초 동안 건드리지 않는다.
- [ ] 기록 이름: `static_30s`
- [ ] 같은 조건에서 2–3회 반복한다(온도 드리프트 확인용).
- [ ] 사람이 테이블을 두드리거나 케이블을 건드리지 않는다.

### 2.2 직진 2 m, 3가지 속도

- [ ] follower를 끄고 텔레옵 또는 `/cmd_vel` 직접 발행으로 움직인다.
- [ ] 바닥에 2 m 구간을 테이프로 표시한다.
- [ ] 속도: **0.06, 0.12, 0.20 m/s** (로봇이 못 내면 0.08, 0.15, 0.22). `v_max`보다 낮게.
- [ ] 각 속도에서 2 m를 직진하고, 끝에서 반드시 0 명령을 보낸다.
- [ ] 기록 이름: `straight_v006_01`, `straight_v012_01`, `straight_v020_01` (각 2회면 `_02`).
- [ ] 실제로 간 거리와 걸린 시간을 적는다. 출발 시점과 2 m 도착 시점의 자세도 가능하면 적는다.

직접 발행 예(끝나면 Ctrl-C 후 0 명령):

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.12}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

### 2.3 제자리 회전 (좌/우)

- [ ] follower를 끈다.
- [ ] 바닥에 시작 방향과 90° 지점을 표시한다.
- [ ] `angular.z = +0.5 rad/s`(좌회전), `-0.5 rad/s`(우회전)로 돌린다.
- [ ] 90°는 약 3.14초, 360°는 약 12.57초 동안 명령한다.
- [ ] 기록 이름: `turn_left_90_01`, `turn_right_90_01`, `turn_left_360_01`.
- [ ] 실제로 돈 각도(바닥 표시로 확인)와 걸린 시간을 적는다. 바퀴 미끄러짐을 보기 위한 자료다.

### 2.4 follower로 한 바퀴

- [ ] `robot.launch.py`로 follower를 켠다. `use_sim_time:=false`인지 확인한다.
- [ ] `v_max`는 낮게(0.10–0.15) 시작한다. 첫 랩은 속도보다 완주가 목적이다.
- [ ] 시작 T에 로봇을 두고 기록을 먼저 시작한 뒤 follower를 켠다.
- [ ] 기록 이름: `lap_follower_01`, `lap_follower_02`.
- [ ] T에서 멈추면 한 바퀴 완료다. 멈추지 않거나 이탈하면 즉시 정지하고, 이탈 지점과 이유를 적는다.
- [ ] 가능하면 2–3바퀴 반복한다. 같은 설정을 유지하고, 설정을 바꾸면 새 run 이름을 쓴다.
- [ ] `ros2 param dump /follower_node > lap_follower_01_params.yaml`로 당시 파라미터를 남긴다.

---

## 3. 끝난 뒤

### 3.1 bag 확인과 정리

- [ ] 각 run의 `ros2 bag info <bag>` 출력을 `bag_info_<run>.txt`로 저장한다.
- [ ] bag 폴더 옆에 `record_run.sh`가 만든 `<run>.yaml`이 있는지 확인한다.
- [ ] 메타데이터에 날짜, 로봇 이름, 배터리, 노트, git commit이 들어 있는지 확인한다.
- [ ] 실패한 run도 지우지 말고 `failed` 표시를 해서 남긴다.

### 3.2 파일 이름과 폴더 구조

```text
bags/real/20261126_tb3/
  20261126_140000_static_30s/
  20261126_140000_static_30s.yaml
  20261126_141000_straight_v006_01/
  20261126_141000_straight_v006_01.yaml
  20261126_141200_straight_v012_01/
  20261126_141200_straight_v012_01.yaml
  20261126_141400_straight_v020_01/
  20261126_141400_straight_v020_01.yaml
  20261126_142000_turn_left_90_01/
  20261126_142000_turn_left_90_01.yaml
  20261126_142100_turn_right_90_01/
  20261126_142100_turn_right_90_01.yaml
  20261126_143000_lap_follower_01/
  20261126_143000_lap_follower_01.yaml
  lap_follower_01_params.yaml
  cam_check_20261126.png
  cam_controls_20261126.txt
  lighting/
  run_log.md
  bag_info_static_30s.txt
  ...
```

`run_log.md`에는 최소한 아래 표를 남긴다.

| run | 시각 | 속도/명령 | 실제 거리/각도 | 결과 | 특이사항 |
|---|---|---|---|---|---|
| static_30s | 14:00 | 0 | – | 성공 | 배터리 11.9 V |
| straight_v006_01 | 14:10 | 0.06 m/s | 2.00 m, 34 s | 성공 | – |
| turn_left_90_01 | 14:20 | +0.5 rad/s | 88° | 성공 | 살짝 미끄러짐 |
| lap_follower_01 | 14:30 | v_max 0.12 | 1바퀴 | 성공 | T에서 정지 |

### 3.3 분석

정지 bag과 직진 bag을 함께 넣는다.

```bash
python3 experiments/real/analyze_static.py \
  bags/real/20261126_tb3/20261126_140000_static_30s \
  bags/real/20261126_tb3/20261126_141000_straight_v006_01 \
  --out bags/real/20261126_tb3/real_analysis.json
```

`--help`로 옵션을 볼 수 있다. `numpy`가 필요하고, bag은 `rosbag2_py`가 있으면 그것으로, 없으면 python `mcap` 패키지나 `.db3` 파일로 읽는다.

출력에서 확인할 것:

- IMU 자이로/가속도 축별 bias와 noise std (`std_diff`는 움직임에 덜 민감한 추정값)
- 정지 시 odom 위치·yaw 드리프트, 직진 시 횡방향 편차와 yaw 변화
- 카메라 평균 rate, 프레임 간격 표준편차, 최대 gap
- IMU와 카메라의 시간 오프셋은 회전 bag과 영상 복호가 있어야 계산할 수 있다. 이 스크립트는 rate와 stamp jitter까지 보고하고, 오프셋은 `not_computed`로 남긴다.

### 3.4 데이터를 넘기는 방법

- [ ] bag, 메타데이터 YAML, 카메라 확인 이미지, 조명 사진, `run_log.md`, 분석 JSON을 한 폴더에 모은다.
- [ ] `build/`, `install/`, `log/`와 다른 사람의 bag은 넣지 않는다.
- [ ] 압축과 체크섬:
  ```bash
  tar -C bags/real -czf real_20261126_tb3.tar.gz 20261126_tb3
  sha256sum real_20261126_tb3.tar.gz > real_20261126_tb3.tar.gz.sha256
  ```
- [ ] 리드가 정한 위치에 올리고, 압축 파일과 체크섬을 함께 전달한다.
- [ ] 함께 알릴 것: 로봇 이름, 카메라 설정(노출/WB), 배터리 상태, run 개수와 실패 내역, 조명 변화, 카메라 장착 실측값(높이/피치/roll), `git commit`.
- [ ] 원본 bag은 지우지 않는다.

---

## 4. 이번에 확인하지 못한 것

`record_run.sh`와 `analyze_static.py`는 이 환경에서 ROS를 실행할 수 없어 `python3 -m py_compile`, 모의 `ros2` 명령, 모의 bag으로만 확인했다. 실제 ROS 2 Jazzy에서 `rosbag2_py`, mcap 플러그인, 카메라 토픽 이름, `/tf_static` QoS는 첫 실행 때 한 번 더 확인한다.
