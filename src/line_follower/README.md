# line_follower

카메라 영상만으로 트랙의 검은 선을 인식해 따라가는 TurtleBot3 Waffle Pi 라인 추종 패키지 (ROS 2 Jazzy, Gazebo Harmonic).

## 실행

```bash
ros2 launch line_follower line_follower.launch.py
```

이 한 줄로 Gazebo(제공 월드, `-r`로 바로 실행), ROS-Gazebo 브리지, 라인 추종 노드가 모두 실행된다. 라인 추종 노드는 launch 후 20초 뒤에 시작한다(`TimerAction`).

선택 인자: `gui:=false` (헤드리스), `start_delay:=<초>`, `params:=<yaml 경로>`.

## 구성

| 파일 | 내용 |
| --- | --- |
| `launch/line_follower.launch.py` | Gazebo + 브리지 + 20초 뒤 노드 시작 |
| `config/bridge.yaml` | `/camera/image_raw`, `/cmd_vel`, `/odom`, `/imu`, `/clock` 브리지 |
| `config/follower.yaml` | 노드 파라미터, `/**: use_sim_time: true` |
| `line_follower/vision.py` | 카메라 영상 → 지면 위에서 본 영상(BEV) → 선 마스크 |
| `line_follower/line_map.py` | odom 좌표계에 고정된 선 지도, 선 추적(교차로 직진, T자 종점 판정) |
| `line_follower/follower_node.py` | 지도 갱신, 경로 계획, pure pursuit 제어 |
| `worlds/`, `models/` | 제공된 월드·모델 (원본과 동일, 수정 없음) |

## 동작 원리

1. **선 인식**: 카메라 높이 103 mm, 차축 앞 76 mm, 수평 화각 62.2°로 투영 변환(homography)을 계산해 영상을 지면 평면으로 펼치고, 밝은 매트보다 충분히 어두운 픽셀을 선으로 본다. IMU 피치로 차체가 앞뒤로 흔들리는 것을 보정한다.
2. **선 지도**: 카메라는 로버 앞 0.31 m 이내를 볼 수 없다. 그래서 매 프레임 본 선을 odom 좌표계 격자 지도에 기록하고, 로버 바로 앞·아래의 선은 지도로 따라간다. 언덕에서 바퀴가 미끄러져 odom 방향이 틀어지는 문제는 방향을 IMU에서 가져와 해결했다.
3. **경로**: 지도에서 로버 앞의 선을 2 cm 간격으로 추적한다. 갈림길에서는 가장 곧은 쪽(교차로는 직진)을 고르고, 좌우로 막대가 있고 앞이 끊긴 곳을 T자 종점으로 판단한다(출발 후 10 m 이상 주행했을 때만).
4. **트랙 맞춤 속도**: `speed_profile`로 주행 거리(odom) 구간마다 최고 속도를 준다. 긴 직선(주행 7.2~10.1 m, 14.0 m 이후)은 0.7 m/s, 루프 출구 뒤 구간은 0.45 m/s, 나머지(언덕·코너가 몰린 구간·루프)는 0.25 m/s다. 종점 T자는 14 m 이상 주행한 뒤에만 인정해 교차로를 종점으로 착각하지 않는다.
5. **제어**: pure pursuit로 조향하고, 앞쪽 경로의 곡률에 맞춰 코너 전에 감속한다. 종점 T자에 도착하면 정지한다.

## 사용한 토픽

로버 토픽만 사용한다: `/camera/image_raw`, `/imu`, `/odom`, `/cmd_vel`. `/world/...` 토픽과 서비스는 사용하지 않는다.

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

인식(`perception/`)과 제어(`control/`)를 플러그인으로 나눴다. `mode:=rule`(기본)은 위의 원본 알고리즘을 그대로 쓰고, `mode:=dl`은 딥러닝 인식을 쓴다(모델 경로 설정 필요). 담당과 실행·채점 방법은 저장소 루트의 `TEAM_GUIDE.md`를 본다.

```bash
ros2 launch line_follower line_follower.launch.py mode:=rule
ros2 launch line_follower line_follower.launch.py mode:=dl
```
