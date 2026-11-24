# line_follower

카메라 영상만으로 트랙의 검은 선을 인식해 따라가는 TurtleBot3 Waffle Pi 라인 추종 패키지 (ROS 2 Jazzy, Gazebo Harmonic).

## 실행

```bash
ros2 launch line_follower line_follower.launch.py
```

이 한 줄로 Gazebo(제공 월드, `-r`로 바로 실행), ROS-Gazebo 브리지, 라인 추종 노드가 모두 실행된다. 라인 추종 노드는 launch 후 20초 뒤에 시작한다(`TimerAction`).

선택 인자: `gui:=false` (헤드리스), `start_delay:=<초>`, `mode:=<모드>`(아래 "모드"), `params:=<yaml 경로>`, `safety:=sim|real`(아래 "안전장치").

## 구성

| 파일 | 내용 |
| --- | --- |
| `launch/line_follower.launch.py` | Gazebo + 브리지 + 20초 뒤 노드 시작 |
| `config/bridge.yaml` | `/camera/image_raw`, `/cmd_vel`, `/odom`, `/imu`, `/clock` 브리지 |
| `config/follower.yaml` | 노드 파라미터, `/**: use_sim_time: true` |
| `line_follower/vision.py` | 카메라 영상 → 지면 위에서 본 영상(BEV) → 선 마스크 |
| `line_follower/line_map.py` | odom 좌표계에 고정된 선 지도, 선 추적(교차로 직진, T자 종점 판정) |
| `line_follower/follower_node.py` | 지도 갱신, 경로 계획, pure pursuit 제어 |
| `line_follower/safety_guard.py` | 유일한 `/cmd_vel` 발행자: 출발 대기, 속도 제한, 센서 끊김·종료 시 정지 |
| `line_follower/situation/` | 상황 분류기(앞쪽 라인 모양 → 11가지 상황), `config/situation_model.yaml` |
| `line_follower/control/playbook.py` | `rule` 제어 + 상황별 대처표 + 종점 판정(`confirm_finish`) |
| `worlds/`, `models/` | 제공된 월드·모델 (원본과 동일, 수정 없음) |

## 동작 원리

1. **선 인식**: 카메라 높이 103 mm, 차축 앞 76 mm, 수평 화각 62.2°로 투영 변환(homography)을 계산해 영상을 지면 평면으로 펼치고, 밝은 매트보다 충분히 어두운 픽셀을 선으로 본다. IMU 피치로 차체가 앞뒤로 흔들리는 것을 보정한다.
2. **선 지도**: 카메라는 로버 앞 0.31 m 이내를 볼 수 없다. 그래서 매 프레임 본 선을 odom 좌표계 격자 지도에 기록하고, 로버 바로 앞·아래의 선은 지도로 따라간다. 언덕에서 바퀴가 미끄러져 odom 방향이 틀어지는 문제는 방향을 IMU에서 가져와 해결했다.
3. **경로**: 지도에서 로버 앞의 선을 2 cm 간격으로 추적한다. 갈림길에서는 가장 곧은 쪽(교차로는 직진)을 고르고, 좌우로 막대가 있고 앞이 끊긴 곳을 T자 종점 후보로 본다. 기본 모드(`playbook_rule`)에서는 상황 분류기가 종점을 확정한다(아래 "모드"). `mode:=rule`에서는 `finish_min_dist`(14 m) 이상 주행했을 때만 인정한다.
4. **트랙 맞춤 속도**: `speed_profile`로 주행 거리(odom) 구간마다 최고 속도를 준다. 긴 직선(주행 7.2~10.1 m, 14.0 m 이후)은 0.7 m/s, 루프 출구 뒤 구간은 0.45 m/s, 나머지(언덕·코너가 몰린 구간·루프)는 0.25 m/s다. `mode:=rule`에서는 종점 T자를 14 m 이상 주행한 뒤에만 인정해 교차로를 종점으로 착각하지 않는다.
5. **제어**: pure pursuit로 조향하고, 앞쪽 경로의 곡률에 맞춰 코너 전에 감속한다. 종점 T자에 도착하면 정지한다.

## 사용한 토픽

로버 토픽만 사용한다: `/camera/image_raw`, `/imu`, `/odom`, `/cmd_vel`. 라인 추종 노드는 명령을 `/line_follower/cmd_raw`로 내고, `safety_guard`가 검사한 뒤 `/cmd_vel`로 보낸다. 가드 상태는 `/line_follower/safety`. `/world/...` 토픽과 서비스는 사용하지 않는다.

## 파라미터 (수업 3-11 Parameter)

모든 값은 `declare_parameter`로 선언하고 범위(`FloatingPointRange`)를 지정했다. `v_max` 0.25 m/s(기본), `speed_profile`(주행 거리별 최고 속도), `a_lat`(코너 횡가속도), `a_dec`(감속), `lookahead_min/gain`(pure pursuit), `finish_min_dist` 등. 실행 중 `ros2 param set /follower_node v_max 0.3` 처럼 바꿀 수 있다(실수는 `0.3`처럼 소수점 포함).

## Docker

```bash
docker build -t line_follower:2 .        # ws/ 폴더(Dockerfile 위치)에서
docker run --rm --device /dev/dri -e DISPLAY=$DISPLAY -e XAUTHORITY=/tmp/.Xauthority \
  -v $XAUTHORITY:/tmp/.Xauthority:ro -v /tmp/.X11-unix:/tmp/.X11-unix line_follower:2
```

베이스 이미지는 수업 실습 이미지 `rp/ros2:jazzy-lab`.

## 모드 (팀 분업 틀)

인식(`perception/`)과 제어(`control/`)를 플러그인으로 나눴다. `mode:=<이름>`이 `config/<이름>.yaml`을 고른다. 담당과 실행·채점 방법은 저장소 루트의 `TEAM_GUIDE.md`를 본다.

| 모드 | 내용 |
| --- | --- |
| `playbook_rule` (기본) | `rule`과 같은 주행 + 학습한 상황 분류기가 종점 T자를 판정(주행 거리 조건 없음) |
| `rule` | 위의 원본 알고리즘 그대로(종점은 14 m 주행 조건) |
| `playbook_tuned` 설정 | 상황별 속도 제한까지 쓰는 실험용(보류 평가에서 더 나빠 기본 아님, `params:=`로 지정) |
| `dl` | 딥러닝 인식(모델 경로 설정 필요) |

```bash
ros2 launch line_follower line_follower.launch.py                 # playbook_rule
ros2 launch line_follower line_follower.launch.py mode:=rule
ros2 launch line_follower line_follower.launch.py mode:=dl
```

**상황 분류기와 종점 판정** (`docs/playbook_learning.md`): 노드가 만든 선 지도·추적 경로·IMU 피치에서 특징 36개를 뽑아 작은 MLP로 상황 11가지(직선, 곡선, 직각, 교차, 루프, 곁가지, 끊김, 언덕, 종점 T, 출발 T, 이탈)를 분류한다(1회 2 ms 미만). T자 후보는 종점 확률 0.6 이상, 막대 너머 0.10 m 이상을 이미 봤고 그 너머에 선이 없으며, 같은 자리에서 3프레임 연속일 때만 종점으로 확정한다. 학습 데이터는 시뮬레이터 정답으로 라벨을 붙였지만(`experiments/situation/`, 개발용), 주행 중에는 로버 센서만 쓴다. 같은 하네스로 보류 맵 58회를 비교했을 때 완주 19회(`rule` 13회), 잘못된 종점 정지 0회(`rule` 4회), 둘 다 완주한 맵의 랩은 같았다(`experiments/results/situation/compare.md`).

## 안전장치 (`safety_guard`)

`/cmd_vel`은 `safety_guard`만 발행한다. 출발 대기(`start_delay`) 동안과 카메라·odom·IMU가 모두 들어오기 전에는 0을 낸다. 속도·각속도를 제한하고, 명령이나 센서가 끊기면(시간 초과) 정지한다. Ctrl-C·SIGTERM에도 0 명령을 보낸 뒤 끝난다. 라인 추종 노드도 종료할 때 0 명령을 보낸다.

| 설정 | 최고 속도 | 최고 각속도 | 센서 시간 초과 |
| --- | --- | --- | --- |
| `safety:=sim` (기본, `config/safety_sim.yaml`) | 1.0 m/s | 4.0 rad/s | 카메라 1.0 s |
| `safety:=real` (`config/safety_real.yaml`, Waffle Pi) | 0.26 m/s | 1.82 rad/s | 0.5 s |

가드 프로세스 자체가 죽으면 Gazebo DiffDrive는 마지막 명령을 계속 유지한다. 실물에서는 구동 드라이버의 명령 시간 초과나 비상 정지 스위치로 막아야 한다. 시험 결과: `experiments/results/safety/`.
