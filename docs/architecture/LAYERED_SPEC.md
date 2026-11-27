# 층별 구조 설계서 (Floorplan 청사진 입력)

2026-11-27 · TurtleBot3 Waffle Pi · Jetson Orin Nano 8GB · Ubuntu 24.04 · ROS 2 Jazzy · 카메라만 (라이다 없음)

베이스: 친구(sion)의 `3abe6cb` (노드 5개 `line_detector`, `path_planner`, `motion_controller`, `safety_guard`, `mission_manager`, 토픽 계약, 안전 불변 조건, CI). 그 위에 우리 인지(Python, `src/line_follower/line_follower/`)를 넣는다. 아래층부터 위층으로 데이터가 흐른다. 같은 내용의 HTML: `docs/architecture/blueprint_layers.html`.

## 층 배치 규칙 (그림)

- 최상위 묶음 6개를 층으로 둔다. 아래에서 위로: **-1층 사전 규칙**, **0층 센서**, **1층 인지**, **1.5층 인지↔판단 경계**, **2층 판단**, **3층 제어**.
- 0층 센서는 왼쪽부터 카메라, IMU, 엔코더 순서.
- 모든 연결선은 아래층에서 위층으로 간다(제어의 /cmd_vel만 0층 OpenCR로 내려간다).
- 각 노드 카드에는 하는 일, 입력, 출력, 결정할 것, 할 일, 담당 팀, 현재 상태를 적는다.

## -1층 사전 규칙 (모든 층이 지킨다)

- **R1 과제 조건:** 한 줄 실행 `ros2 launch line_follower_bringup competition.launch.py`. 시작 후 20초 대기 뒤 자동 출발(Gazebo 과제). 제공 월드·로봇 모델 수정 금지(SHA256). 카메라 영상으로 선 인식. 로버 토픽만 사용(`/camera/image_raw`, `/imu`, `/odom`, `/cmd_vel`). `/world/` 토픽 금지.
- **R2 안전 불변 조건:** `safety_guard`만 `/cmd_vel`을 낸다. 미션 허가가 없으면 속도 0. 영상은 촬영 시각 기준으로 odom과 맞춘다. 오래된 영상·경로·후보는 버린다. 실시간 루프는 막히지 않고, GCS·기록이 죽어도 주행에 영향이 없다.
- **R3 좌표·단위·시간:** odom 좌표(x 전방, y 왼쪽), m·rad·s. 지도 5 mm 격자. BEV 0.31–0.85 m(먼 띠 1.3 m) × ±0.55 m. Jetson 카메라 프레임 예산 33 ms(지금 TensorRT로 21–26 ms). 실물은 `use_sim_time:=false`, 라이다 없음.
- **R4 팀 규칙:** 인지는 칠해진 선을 가짜 길까지 모두 넘기고, 길 선택은 판단이 한다. 인지 평가는 지도 정밀도·재현율과 중심선 오프셋으로 한다. 공식 트랙 과적합 금지. 버전 이름은 `V<n>-부품해상도-데이터`(`docs/VERSIONS.md`). `rule.yaml` 키 변경은 `-p follower_node:키:=값`. CI 통과 후 PR, push는 승인 후.

## 0층 센서 (왼쪽부터)

| 센서 | 공식 사양 | 우리가 쓰는 범위 | 토픽 |
|---|---|---|---|
| **S1 카메라** Raspberry Pi Camera v2.1 (Sony IMX219), CSI CAM0, 금색 단자 아래 | 8 MP 3280×2464, 픽셀 1.12 µm, 센서 3.68×2.76 mm, 초점 3.04 mm, F2.0, 수평 62.2° 수직 48.8°. 모드 3280×2464@21, 3280×1848@28, 1920×1080@30, 1640×1232@30, 1280×720@60 | 1640×1232@30(전체 화각) → 640×480 bgr8 30 Hz. 내부값 이론 fx=fy≈529.6 px, cx≈319.5, cy≈239.5(시뮬 530.5와 일치, 왜곡은 보정). 장착: 차축 앞 0.076 m, 높이 0.103 m, 피치 0. 화면 아래 끝이 보는 땅은 차축 앞 0.30 m(그 안은 사각지대) | `/camera/image_raw`, `/camera/camera_info` |
| **S2 IMU** OpenCR 1.0 내장 | MPU9250(초기) 또는 ICM-20648(2020년 이후), 6축. 공식 주기 없음(170–200 Hz 언급) | 카메라 피치(언덕 투영), 언덕 마루 바퀴 미끄러짐 보정, 안전 기울기. 주기·잡음은 실측 | `/imu` |
| **S3 바퀴 엔코더** Dynamixel XM430-W210-T × 2 | 절대 엔코더 4096 스텝/회전, 무부하 77 rpm@12 V, 70 rpm@11.1 V | 1스텝 0.051 mm(바퀴 반지름 0.033 m). 무부하 최고 0.266 m/s@12 V, 0.242 m/s@11.1 V. 공식 상한 0.26 m/s, 1.82 rad/s. 바퀴 간격 0.287 m | `/odom`(odom→base_footprint), `/joint_states` |
| OpenCR 1.0 + turtlebot3_node (출력 쪽) | 모터·IMU·배터리 보드 | `/cmd_vel`을 받아 모터 구동, `/odom` `/imu` 발행 | `/cmd_vel` 입력 |

## 1층 인지 (인지 팀)

### N1 camera_driver (실물 전용)
- 하는 일: `nvarguscamerasrc` 모드 1640×1232@30 → `nvvidconv`로 640×480 축소 → 발행.
- 입력: IMX219 원시 영상(CSI CAM0). 출력: `/camera/image_raw` 640×480 bgr8 30 Hz(촬영 시각 stamp), `/camera/camera_info`.
- 결정: 강의 래퍼(packbionics csi-camera-ros-wrapper 수정) vs 자체 노드, 노출·화이트밸런스 고정 여부.
- 할 일: 카메라 물리 연결(지금 probe -121) → 30 Hz 확인 → 체커보드 보정. 상태: 할 일. jetson-io IMX219-A는 적용함.
- 파일(계획): `src/line_follower_camera/` (launch, params, 필요시 노드).

### N2 line_detector (베이스 노드 이름, 우리 Python 인지 플러그인)
- 하는 일: 가까운 곳 규칙 임계값(V0 640, V1 320) 또는 학습 모델 선 분할+깊이(V2, TensorRT), 혼합(규칙을 DL로 걸러 냄), 그림자 폭 필터, 먼 곳 640 지평선 띠 모델, 언덕 IMU 피치+지면 추정.
- 입력: `/camera/image_raw`, 촬영 시각의 Pose와 피치(1.5층 N4).
- 출력: `/perception/bev` = BEV 선 마스크 108×220(0.31–0.85 m, 먼 띠는 1.3 m까지), 유효 영역, 칸 신뢰도, 촬영 시각, 그 시각 Pose. `/perception/status` = 프레임 지연, 인지 버전, 백엔드(TensorRT/CPU), 폴백.
- 결정: 기본 버전(지금 V0-NearRule640), 조명 강건화 방법(폭 필터·구역별 임계·혼합), 먼 띠 사용(0.85 m vs 1.3 m).
- 할 일: 실물 카메라 영상으로 재검증, 규칙 파라미터 최적화 결과 반영. 상태: 있음.
- 파일: `perception/stage.py`, `perception/rule.py`, `perception/depth.py`, `perception/hybrid.py`, `vision.py`, `terrain.py`, `far_band.py`, `trt_runner.py`.

## 1.5층 인지↔판단 경계 (코드는 인지 소유, 실행은 판단과 같은 프로세스)

### N5 line_map (Local Path Memory)
- 하는 일: 본 것을 odom 5 mm 격자에 촬영 시각 자세로 투영해 누적. 층: 선·본 영역·신뢰도·복원·고정. 누적 방식 legacy 또는 logodds.
- 입력: `/perception/bev` + 촬영 시각 Pose. 출력: 읽기 전용 `MapView`(hit·snap·trace·confidence·seen) → N7, `/lf/map`(OccupancyGrid 2 Hz, 4 m×4 m, 1 cm).
- 왜 1.5층: 베이스 설계는 Local Path Memory를 판단 노드 안에 두지만, 지도 정확도는 인지의 성적이다.
- 결정: 기본 누적 방식(logodds가 기본 조명 정밀도 0.961→0.978, 재현율 유지 — 폐루프 판정 대기), 지나간 공간 고정 사용 여부. 상태: 있음. 파일: `line_map.py`.

### N4 state_estimator (위치 추정)
- 하는 일: odom 기록·보간, IMU 피치, 언덕 마루 미끄러짐 보정, 지도 정합 보정 적용.
- 입력: `/odom`, `/imu`, 지도 정합 보정량. 출력: `/localization/estimate`, 함수 `pose_at(t)` → `Pose(t, x, y, yaw, pitch, travel)`.
- 왜 1.5층: 베이스는 위치 추정을 판단에 주고, 우리는 지도 투영에 쓴다. 양쪽이 같은 자세를 봐야 한다.
- 결정: `Pose.pitch` = 걸러진 피치, 영상·자세 시간 허용폭. 상태: 노드 안 함수 → 분리 예정. 파일: `state/odom_imu.py`, `scanmatch.py`.

### N6 restorer (안 본 곳 복원)
- 하는 일: 선 지도 주변 128×128 패치로 안 본 선을 이어 `restored` 층(신뢰도 0.31)에 쓴다.
- 왜 1.5층: 인지가 선을 채울지 판단이 경로를 늘릴지 애매 → 인지가 쓰고 판단은 읽기만.
- 결정: 켤지 여부(공식 트랙에서 손해라 끔). 파일: `restore.py`.

## 2층 판단 (판단 팀)

### N7 path_planner (베이스 노드)
- 하는 일: 선 따라가기(trace), 교차로·가짜 길 가지 선택(규칙 또는 MLP), 짧은 오검출 조각 무시, 결승 T 판정, 구간 속도 상한.
- 입력: `MapView`, Pose. 출력: `/path/local` = odom 점 목록(약 2 cm), 끝 상태 open/t/len, 점별 신뢰도, 속도 상한, T 지점. `/path/status` = 유효성, 모드, 검증된 남은 거리, 결승 이벤트. `/path/candidates`(개발용).
- 결정: 가지 선택 규칙(지금 가장 곧은 가지), 신뢰도 낮은 구간 감속, 결승 확정 조건(출발 T 오인 방지).
- 할 일: decoy1000 시험 맵에서 가지 선택 정확도. 상태: Python RulePlanner 있음, 베이스 C++ 있음. 파일: `planning/rule.py`, `planning/base.py`.

### N8 mission_manager (베이스 노드)
- 하는 일: 상태 머신 STARTUP_WAIT → CHECKING → READY → RUNNING → STOPPING → FINISHED (FAULT, E_STOP).
- 입력: 센서 상태, `/path/status` 결승 이벤트. 출력: `/mission/state`, 주행 허가.
- 결정: 실물 평가 시작 방식(20초 대기 유지 여부), 완주 확정 규칙. 상태: 베이스 C++ 있음. 패키지: `line_follower_mission`.

## 3층 제어 (제어 팀)

### N9 motion_controller (베이스 노드, 50 Hz)
- 하는 일: Pure Pursuit + 횡오차 PID, 곡률·신뢰도·언덕 속도 프로파일, 급코너 정지 후 회전.
- 입력: `/path/local`, Pose. 출력: `/control/cmd_vel_candidate` = v, w, 계산 시각, 유효 기한.
- 결정: lookahead·PID 게인, 실물 속도 상한(무부하 최고 0.24 m/s@11.1 V → 0.22 m/s 안팎), 가감속 한계.
- 할 일: 실물 저속 0.10–0.15 m/s 튜닝 후 상승. 결승 T 정지 실행만(판정은 2층). 상태: Python RuleController 있음, 베이스 C++ 있음. 파일: `control/rule.py`.

### N10 safety_guard (베이스 노드)
- 하는 일: 유일한 `/cmd_vel` 발행. 허가 없으면 0, 오래된 데이터 차단, 속도·가속·각속도 제한, 이상 시 정지.
- 입력: 명령 후보, `/mission/state`, `/path/status`, 센서 신선도, `/imu` 기울기. 출력: `/cmd_vel` → turtlebot3_node → OpenCR.
- 결정: 타임아웃 값, 제한값, 실물 E-STOP·워치독. 할 일: 우리 Python 파이프라인 출력을 이 노드로 연결. 상태: 베이스 C++ 있음, 미연결. 패키지: `line_follower_control`.

## 연결 (아래 → 위)

| 보내는 곳 | 받는 곳 | 통로 | 내용 | 주기 |
|---|---|---|---|---|
| S1 카메라 | N1 | CSI CAM0 | IMX219 원시 영상, 1640×1232 | 30 Hz |
| N1 | N2 | `/camera/image_raw` | 640×480 bgr8, 촬영 시각 | 30 Hz |
| S2 IMU, S3 엔코더 | N4 | `/imu`, `/odom` | 각속도·가속도 / 자세·속도 | IMU 실측, odom ≥20 Hz |
| N4 | N2, N5, N7, N9 | `Pose`(`pose_at(t)`), `/localization/estimate` | t, x, y, yaw, pitch, travel | 요청 시 / 30 Hz |
| N2 | N5 | `/perception/bev` | BEV 선 마스크·유효 영역·칸 신뢰도(108×220 uint8), 촬영 시각, Pose | 30 Hz |
| N2 | N10, 개발 도구 | `/perception/status` | 지연, 버전, 백엔드, 폴백 | 2 Hz |
| N5 (+N6) | N7 | `MapView`(같은 프로세스) | hit·snap·trace·confidence·seen | 프레임마다 |
| N5 | RViz·GCS | `/lf/map` | OccupancyGrid 4 m×4 m 1 cm | 2 Hz |
| N7 | N9 | `/path/local` | 점 목록, 끝 상태, 신뢰도, 속도 상한, T 지점 | 30 Hz |
| N7 | N8, N10 | `/path/status` | 유효성, 모드, 남은 거리, 결승 이벤트 | 30 Hz |
| N8 | N10, 전체 | `/mission/state` | 단계, 주행 허가, 종료 사유 | 변경 시 + 2 Hz |
| N9 | N10 | `/control/cmd_vel_candidate` | v, w, 시각, 유효 기한 | 50 Hz |
| N10 | OpenCR(0층) | `/cmd_vel` | 최종 Twist(유일 발행자) | 50 Hz |

배치: 논리 노드는 위와 같고, 실행은 베이스 설계서 4.3대로 컴포저블 노드로 묶을 수 있다. N2–N7은 같은 프로세스에서 함수 호출로 이어 붙인다(지도 복사 없음). N8–N10과 카메라 드라이버는 별도 노드.
