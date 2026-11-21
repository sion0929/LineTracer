# Project #1 Line Follower 로버 — 실행 방법 (제출용)

- 과목: Robot Programming 프로젝트 #1 · 충북대학교 지능로봇공학과
- 학번/이름: 2022042036 천시온
- 학번/이름:  이진혁
- 학번/이름:  유하린
- 환경: ROS 2 Jazzy, Gazebo Harmonic, 기반 이미지 `rp/ros2:jazzy-lab`

## 실행 명령어 (한 줄)

```bash
ros2 launch line_follower_bringup competition.launch.py
```

이 한 줄이 Gazebo(제공 월드·로버 모델 원본 그대로), ROS–Gazebo 브리지, 라인 추종 프로그램을 모두 실행합니다.
라인 추종은 실행 후 **20초 뒤** 자동으로 시작합니다. `/world/` 토픽·서비스는 사용하지 않고 로버 토픽
(`/camera/image_raw`, `/imu`, `/odom`, `/cmd_vel`)만 사용합니다.

## Docker로 실행 (권장)

이 폴더(`Dockerfile`이 있는 곳)에서:

```bash
docker build -t line_follower:latest .
xhost +local:docker
docker run --rm -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix line_follower:latest
```

컨테이너의 기본 명령이 위의 한 줄 실행 명령입니다. 직접 입력하려면:

```bash
docker run --rm -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix line_follower:latest \
  ros2 launch line_follower_bringup competition.launch.py
```

GUI 없이 실행: 명령 끝에 `headless:=true`.

## 직접 빌드하여 실행 (rp/ros2:jazzy-lab 등 ROS 2 Jazzy 환경)

```bash
colcon build --packages-up-to line_follower_bringup
source install/setup.bash
ros2 launch line_follower_bringup competition.launch.py
```

## 폴더 구성

| 경로 | 내용 |
|---|---|
| `src/line_follower_bringup` | 실행 launch, 파라미터, 브리지 설정, **제공 worlds/models 원본**(SHA256SUMS로 무결성 확인) |
| `src/line_follower_perception` | 카메라 라인 검출(C++ OpenCV) |
| `src/line_follower_navigation` | 로컬 경로 기억·경로 판단(path_planner) |
| `src/line_follower_control` | 제어기(motion_controller), 안전 계층(safety_guard, 유일한 /cmd_vel 발행) |
| `src/line_follower_mission` | 20초 대기·미션 상태(mission_manager) |
| `src/line_follower_interfaces` | 메시지/서비스 정의 |
| `src/line_follower_tests` | 자동 시험 |
| `tools/` | 개발용 도구(GCS, 검사 스크립트) — 평가 실행에는 필요 없음 |
| `docs/` | 설계서·검증 기록 |
| `Dockerfile`, `docker/` | 평가용 Docker 환경 |
