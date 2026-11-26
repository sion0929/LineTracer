# 실물 TurtleBot3 + Jetson 세팅 (Lab 09 자료 정리 + 우리 스택에 맞춘 차이점)

2026-11-26 · 출처: 강의 자료 "Robot Programming Lab 09: Jetson orin, turtlebot setting" (22쪽)


## 0. 강의 영상(22분, 무음, 자막)에서 확인한 사실

| 항목 | 영상 근거 |
|---|---|
| 강의용 Jetson OS | `lsb_release -a` → **Ubuntu 22.04.5 LTS (jammy)**, ROS 2 **Humble** 설치 문서를 따라감 (12:22) |
| OS 설치 | 보드의 **REC와 GND 핀을 점퍼로 연결**(복구 모드) → SDK Manager를 설치한 호스트 PC와 **USB-C 케이블**로 연결 → SD 카드 슬롯에 카드 → SDK Manager로 설치 (2:30–3:20) |
| 조립 | Jetson 아래판 나사를 풀어 분리, NIC 안테나 선 정리 후 TurtleBot3 판에 장착. OpenCR와 Jetson을 USB로 연결 (4:47–7:14) |
| **카메라 연결** | Jetson에는 CAM0, CAM1 두 포트가 있다. **CAM0 포트에 금색 단자가 아래쪽을 향하도록** 리본 케이블을 꽂는다 (7:35–7:44) |
| jetson-io | 24pin CSI Connector → Camera IMX219-A → 저장 후 재부팅 (8:26–9:16) |
| **IMX219 센서 모드 (실측 출력)** | `3280×2464 @21 fps`, `3280×1848 @28`, `1920×1080 @30`, **`1640×1232 @30`**, `1280×720 @60` (11:28). JetsonHacks 예제는 mode 2 (1920×1080, 16:9, 화각 잘림)로 연다 |
| 화면 테스트 | `export DISPLAY=:1`이 꼭 필요하다는 자막 (13:43) |
| TurtleBot3 구동 | `TURTLEBOT3_MODEL`, `LDS_MODEL` export는 `.bashrc`에 넣기 권장, `robot.launch.py` 후 새 터미널에서 `teleop_keyboard`로 확인 (20:57–21:46) |

→ 우리 노드에는 **mode 3 (`1640×1232 @30`, 4:3 전체 화각)**을 `640×480`으로 줄여 쓴다.

## 1. 강의 자료의 순서

| 단계 | 어디서 | 명령 / 내용 |
|---|---|---|
| Jetson 이미지 | 호스트 PC | NVIDIA SDK Manager 2.3.0 (`sudo dpkg -i sdkmanager_2.3.0-12626_amd64.deb`)으로 Jetson에 OS 설치 |
| CSI 카메라 켜기 | Jetson | `sudo /opt/nvidia/jetson-io/jetson-io.py` → Configure Jetson 24pin CSI Connector → Configure for compatible hardware → **Camera IMX219-A** → Save pin changes → Save and reboot. `/boot/extlinux/extlinux.conf`에 `tegra234-p3767-camera-p3768-imx219-dual.dtbo`가 추가됨 |
| 카메라 확인 | Jetson | `ls -al /dev/video*` → `/dev/video0` |
| 화면 테스트 (모니터 필요) | Jetson | `sudo apt install v4l-utils`, `git clone https://github.com/JetsonHacksNano/CSI-Camera`, `export DISPLAY=:1`, `python3 simple_camera.py` |
| ROS 카메라 발행 (모니터 필요) | Jetson | `git clone https://github.com/packbionics/csi-camera-ros-wrapper`, `ros2 launch image_pub raw_image_pub.launch.py` |
| OpenCR 펌웨어 | Jetson | `dpkg --add-architecture armhf`, `apt install libc6:armhf`, `OPENCR_PORT=/dev/ttyACM0`, `OPENCR_MODEL=waffle`, `opencr_update.tar.bz2` 받아서 `./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr` |
| OpenCR 모터 설정 | 호스트 PC | udev 규칙 `99-opencr-cdc.rules`, `libncurses5-dev:i386`, Arduino IDE 2.3.3 + OpenCR 보드 패키지 1.5.3 + Dynamixel2Arduino, `sudo apt-get purge modemmanager`, 예제 `turtlebot3_setup_motor` 업로드 → 시리얼 모니터에서 1–4 (왼쪽·오른쪽 모터 설정·시험, 배터리 연결) |
| TurtleBot3 패키지 | Jetson | `ros-humble-*` (hls-lfcd-lds-driver, xacro, turtlebot3-msgs, dynamixel-sdk), `libudev-dev`, `turtlebot3 -b humble`, `ld08_driver -b ros2-devel`, `colcon build`, udev `99-turtlebot3-cdc.rules` |
| 구동 | Jetson | `export TURTLEBOT3_MODEL=waffle`, `export LDS_MODEL=LDS-02`, `ros2 launch turtlebot3_bringup robot.launch.py`, 시험 `ros2 run turtlebot3_teleop teleop_keyboard` |

## 2. 우리 스택과 다른 점 (꼭 맞춰야 할 것)

| # | 항목 | 강의 자료 | 우리 | 해야 할 일 |
|---|---|---|---|---|
| 1 | **OS / ROS** | Ubuntu 22.04 (커널 5.15-tegra) + **ROS 2 Humble** | E의 Jetson: Ubuntu 24.04.5, L4T R39.2.2 + **ROS 2 Jazzy**. 코드도 Jazzy 기준 | 로봇에 올릴 보드가 어느 이미지인지 정해야 한다. Jazzy면 `ros-jazzy-*`, `turtlebot3 -b jazzy`로 바꾼다. Humble이면 우리 노드를 Humble에서 시험해야 한다 |
| 2 | **카메라 토픽** | 래퍼 기본 설정은 스테레오 카메라(IMX219-83)용. `namespace: /camera/left` → `/camera/left/image_raw` | 노드는 `/camera/image_raw`를 구독 | 설정 YAML을 우리 것으로: `namespace: /camera`, `sensor_id: 0` (IMX219-A 슬롯), `frame_id: camera_rgb_frame` |
| 3 | **주기** | `framerate: 20` | 30 Hz 기준 설계 | 30으로. 다만 아래 4번 때문에 센서 모드도 바꿔야 30 Hz가 나온다 |
| 4 | **센서 모드 / 화각** | 래퍼가 `3264×2464` 전체 해상도 모드로 열고 `640×480`으로 줄임. IMX219의 이 모드는 최대 약 21 fps | 30 Hz, 4:3 전체 화각 필요 | 래퍼의 GStreamer 문자열을 `1640×1232`(2×2 비닝, 4:3 전체 화각, 30 fps) 모드로 바꾼다. 16:9 모드(1920×1080, 1280×720)는 화각이 잘려 쓰지 않는다. 강의 영상의 실측 모드 목록에 `1640×1232 @30`이 있다 |
| 5 | **카메라 내부 파라미터** | 래퍼 YAML의 `fx 455.6` 등은 다른 카메라 값 (640 폭에서 수평 화각 약 70°) | `vision.py`는 수평 화각 62.2° (`fx ≈ 530`)를 가정 | 체커보드로 우리 카메라를 보정한다. 결과를 `vision.py`(또는 실물용 파라미터)에 넣는다. 왜곡 계수도 반영 |
| 6 | **영상 형식** | `bgr8`, 시각은 발행 순간의 벽시계 | 노드는 `rgb8`/`bgr8`/`mono8` 모두 받음 | 그대로 됨. 실물은 `use_sim_time:=false` (`experiments/jetson/robot.launch.py`) |
| 7 | **라이다** | `ld08_driver` 설치, `LDS_MODEL=LDS-02`, `robot.launch.py`가 라이다도 띄움 | **라이다 없음** | 라이다 노드를 빼고 `turtlebot3_node`(OpenCR: `/odom`, `/imu`, `/cmd_vel`)만 띄우는 launch를 쓴다. 라이다 장치가 없으면 그 노드는 오류를 내며 재시작을 반복할 수 있다 |
| 8 | **모델 이름** | `TURTLEBOT3_MODEL=waffle`, `OPENCR_MODEL=waffle` | Waffle Pi | OpenCR 펌웨어는 `waffle`이 맞다(Waffle Pi도 같은 펌웨어). bringup 파라미터는 `waffle_pi`가 있으면 그것을 쓴다 (확인 필요) |
| 9 | **모니터** | 카메라 시험에 모니터 필요 (`DISPLAY=:1`) | E는 SSH로만 접속 | 화면 없이 확인하려면 `gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=30 ! fakesink` 또는 한 장 저장(`nvjpegenc ! filesink`)으로 한다 |

## 3. 실물에서 우리 노드까지 순서 (제안)

1. 보드 이미지와 ROS 배포판 확정 (위 1번).
2. OpenCR 펌웨어와 모터 설정 (강의 자료 그대로). `teleop_keyboard`로 바퀴가 도는지 확인.
3. 라이다 없는 bringup으로 `/odom`, `/imu` 확인: `ros2 topic hz /odom /imu`.
4. jetson-io로 IMX219-A 켜기 → 재부팅 → `/dev/video0` → GStreamer로 한 장 저장.
5. 카메라 래퍼를 우리 설정(위 2–4번)으로 띄우기 → `ros2 topic hz /camera/image_raw` 30 Hz, 640×480 확인.
6. 체커보드 보정, 장착 높이·피치 실측 → 실물 파라미터.
7. `experiments/jetson/robot.launch.py`로 인지 노드 실행, `/line_follower/debug`로 선 검출 확인 (바퀴 끈 상태).
8. `docs/REAL_ROBOT_TEST.md` 순서로 정지·직진 기록 → 잡음 측정 → 저속 주행.

## 4. 시스템 변경 주의

jetson-io 설정, 펌웨어 업데이트, apt 설치, `modemmanager` 삭제는 보드 상태를 바꾼다. 실행 전 사용자 승인을 받고, jetson-io 전에는 `/boot/extlinux/extlinux.conf`를 백업한다.
