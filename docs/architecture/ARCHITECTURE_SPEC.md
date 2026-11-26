# 라인 트레이서 노드 구조 설계서 (목표 구조)

2026-11-26 · TurtleBot3 Waffle Pi · ROS 2 Jazzy · Jetson Orin Nano 8GB · 마감 2026-12-09

## 0. 이 문서의 목적

세 팀(인지, 판단, 제어)이 서로의 코드를 건드리지 않고 동시에 개발할 수 있도록 노드의 뼈대를 정한다. 지금 코드는 대부분 동작하지만 경계가 흐리다. `follower_node.py` 한 파일(430줄)이 ROS 입출력, 자세 추정, 단계 호출, 디버그 출력을 모두 한다. 이 문서는 무엇을 어디에 두고, 단계 사이에 무엇을 주고받는지 정한다.

## 0-1. 플랫폼과 입력 센서

**센서는 카메라 하나, IMU, 바퀴 엔코더(odom)뿐이다. 라이다는 없다.** 인지는 카메라만으로 선을 찾는다. IMU와 엔코더는 자세 추정에만 쓴다.

### 로봇과 컴퓨터

| 항목 | 실물 | 근거 |
|---|---|---|
| 로봇 | ROBOTIS TurtleBot3 Waffle Pi (라이다 없음) | 팀 구성 |
| 주행 한계 | 직진 0.26 m/s, 회전 1.82 rad/s | ROBOTIS 공식 사양 |
| 구동 | Dynamixel XM430-W210 2개, 차동 구동. 바퀴 반지름 0.033 m, 바퀴 간격 0.287 m | ROBOTIS 사양, 로봇 모델 파일 |
| 제어 보드 | OpenCR 1.0 (모터, IMU, 배터리) | ROBOTIS 사양 |
| 주 컴퓨터 | **Jetson Orin Nano 8GB Developer Kit** (Super). CPU 6코어, GPU Ampere 1024코어 + Tensor 코어 32개, RAM 7.5 GB, swap 2 GB | E 실측, NVIDIA 사양 |
| 소프트웨어 | Ubuntu 24.04.5, L4T R39.2.2, ROS 2 Jazzy, CUDA 13.2, TensorRT 11.4.1.31, OpenCV 4.6.0. onnxruntime 없음 | E 실측 |
| 전력 모드 | MAXN_SUPER, 클럭 동적. 유휴 3.9 W, 노드 실행 중 5.3–5.4 W (보드 입력) | E 실측 |

### 입력 센서

| 센서 | 토픽 | 실물 | 시뮬레이션 (Gazebo 모델) | 인지가 쓰는 방식 |
|---|---|---|---|---|
| **카메라** | `/camera/image_raw` (`sensor_msgs/Image`, rgb8) | 얇은 납작 리본 케이블(**CSI, USB 아님**)로 연결. Waffle Pi 기본 Raspberry Pi Camera v2.1 (Sony IMX219, 8 MP)로 추정. Jetson 쪽은 22핀 CSI라 15핀→22핀 케이블이 필요하고, jetson-io로 IMX219를 켜야 `/dev/video*`가 생긴다 (E의 보드에서 아직 없음). RAW Bayer라 `nvarguscamerasrc`(ISP)로 받아 640×480 rgb8로 줄여 발행해야 한다 | 640×480, 30 Hz, 수평 화각 1.0856 rad (62.2°), 픽셀 잡음 σ 0.007 | 640×480 BGR로 받아 320×240(가까운 곳) 또는 640 원본(먼 띠)을 쓴다. 30 Hz 기준으로 설계 |
| 카메라 장착 | `camera_rgb_frame` | 실측 확인 필요 | 차축 앞 0.076 m, 지면 위 0.103 m, 수평 (피치 0) | `vision.py`의 `CAM_X`, `CAM_H`, `HFOV`. 실물에서 장착 높이와 피치를 재서 맞춘다 (±2°, ±5 mm 오차를 학습 데이터에 넣음) |
| **IMU** | `/imu` (`sensor_msgs/Imu`) | OpenCR 1.0 내장 IMU (가속도, 자이로). 칩과 실제 주기 확인 필요. 체크리스트 기준 100 Hz 이상 | 200 Hz, 자이로 σ 2e-4 rad/s, 가속도 σ 1.7e-2 m/s² | 카메라 피치 추정(언덕), 언덕 마루에서 바퀴 미끄러짐 보정 |
| **엔코더 odom** | `/odom` (`nav_msgs/Odometry`, `odom` → `base_footprint`) | turtlebot3_node가 Dynamixel 엔코더로 계산. 체크리스트 기준 20 Hz 이상 | 30 Hz, 차동 구동 플러그인 (잡음 없음) | 로봇 자세, 지도 좌표계의 기준. 실물의 반지름 오차, 미끄러짐은 나중에 잡음 시험 |
| 출력 | `/cmd_vel` (`geometry_msgs/Twist`) | turtlebot3_node → OpenCR | 차동 구동 플러그인 (가속 한계 1.0 m/s²) | 제어 단계가 50 Hz로 낸다 |

실물 세팅 절차와 강의 자료와의 차이(ROS 배포판, 카메라 래퍼 토픽·주기·센서 모드·내부 파라미터, 라이다 제외)는 `docs/REAL_ROBOT_SETUP.md`에 있다. 시뮬레이션 값은 `src/line_follower_bringup/models/turtlebot3_waffle_pi/model.sdf`와 `vision.py`에 있다. 실물 잡음 측정 절차는 `docs/REAL_ROBOT_TEST.md`에 있다.

### 실물에서 확인할 것

1. **CSI 카메라 켜기와 발행.**
   - 카메라 모델(IMX219 v2.1 여부)을 확인한다.
   - jetson-io로 IMX219를 켠다. 부팅이 안 되는 사례가 보고되어 있으니 먼저 백업하고, 사용자 승인 뒤에 한다.
   - `gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! fakesink`로 확인한다.
   - ROS 2 발행 노드를 정한다 (GStreamer 기반 gscam2 또는 자체 노드).
   - **센서 모드는 4:3 전체 화각 모드**(예: 1640×1232 비닝)를 골라 640×480으로 줄인다. 16:9 모드는 화각이 잘려 시뮬레이션의 62.2°와 달라진다. 실제 모드 목록은 보드에서 조회한다.
   - 내부 파라미터를 보정한다(`camera_info`). 주기와 지연을 잰다.
2. 카메라 장착 높이, 앞 거리, 피치를 실측해 `vision.py` 상수와 맞춘다.
3. IMU 칩, 주기, 잡음. 정지 기록으로 잰다.
4. odom 주기와 바퀴 반지름 오차. 직진 기록으로 잰다.
5. 실물 시간 동기화: `use_sim_time:=false`, 카메라 타임스탬프 지연.

## 1. 큰 결정

1. **ROS 노드는 하나(`follower_node`, Python)로 유지한다.** 단계는 같은 프로세스 안의 플러그인으로 부른다.
   - 이유: 선 지도는 2400×2400칸(5 mm)이다. 매 프레임 토픽으로 넘기면 Jetson에서 지연과 복사 비용이 크다.
   - Jetson 실측: 한 프로세스에서 프레임 처리 p95가 21–25 ms이고, 예산은 33 ms다.
2. **단계 사이 인터페이스는 파이썬 데이터 클래스로 고정한다.** 팀은 자기 폴더 안에서만 구현을 바꾼다.
3. **관찰용 ROS 토픽을 낸다.** 지도, 경로, 디버그 영상이다. RViz로 보고 bag으로 기록할 수 있게 한다. 나중에 노드를 나눠야 하면 이 토픽이 그 경계가 된다.
4. **제출 이미지에 들어가지 않는 것은 패키지 밖에 둔다.** 실험 도구, 오라클, 벤치는 `experiments/`에 둔다. 기존 C++ 다중 노드 패키지(`line_follower_perception` 등)는 쓰지 않는 레거시로 표시한다.

## 2. 처리 흐름

```
/camera/image_raw (30 Hz) ─┐
/odom (~30 Hz) ────────────┼─> [상태 추정 State] ──> Pose(t)
/imu (~200 Hz) ────────────┘          │
                                       ▼
카메라 콜백: [인지 Perception] ──> LineMap 갱신 ──> MapView(읽기 전용)
                                       │
                                       ▼
             [판단 Planning] ──> LocalPath (odom 점 목록, 끝 상태, 결승 T)
                                       │
제어 타이머 50 Hz: [제어 Control] ──> Command(v, w) ──> /cmd_vel
                                       │
관찰 타이머 2 Hz:  [관찰 Telemetry] ──> /lf/map, /lf/path, /lf/debug
```

## 3. 모듈과 담당

| 모듈 | 위치 (패키지 `line_follower`) | 담당 | 하는 일 |
|---|---|---|---|
| 오케스트레이터 | `node.py` (지금 `follower_node.py`) | 공통 | ROS 구독·발행, 파라미터, 타이머, 단계 연결만. 계산은 하지 않는다 |
| 상태 추정 | `state/` (`StateEstimator`) | 인지 | odom 기록, IMU 피치, 미끄러짐 보정, 시각별 자세 조회 `pose_at(t)`, `pitch_at(t)`. 지금 노드 안의 `on_odom`, `on_imu`, `pose_at`, `pitch_at`, `scanmatch.py`를 옮긴다 |
| 인지 | `perception/` (`PerceptionStage` + 검출 플러그인 rule / depth / hybrid / dl) | 인지 | 영상 → BEV 선 마스크 → 지도 갱신. 지형(`terrain.py`), 먼 띠(`far_band.py`), 복원(`restore.py`), TensorRT(`trt_runner.py`), 카메라 기하(`vision.py`) 포함 |
| 지도 | `perception/line_map.py` (`LineMap`) + `MapView` | 인지 | odom 고정 격자. 판단에는 읽기 전용 `MapView`만 넘긴다 |
| 판단 | `planning/` (`Planner` 플러그인: rule …) | 판단 | MapView + Pose → LocalPath. 교차로·가짜 길 선택, 결승 T 인식 |
| 제어 | `control/` (`Controller` 플러그인: pure pursuit + PID, learned) | 제어 | LocalPath + Pose → Command. 속도 프로파일, 결승 정지 |
| 관찰 | `telemetry.py` | 공통 | 지도 일부를 `nav_msgs/OccupancyGrid`로, 경로를 `nav_msgs/Path`로, 디버그 영상을 `sensor_msgs/Image`로 발행 |
| 설정 | `config/` | 각 팀 | `perception.yaml`, `planning.yaml`, `control.yaml`, `state.yaml`을 launch가 합친다. 버전 묶음은 `config/profiles/V0-NearRule640.yaml` 같은 파일로 |

## 4. 단계 사이 데이터 (파이썬 데이터 클래스, `line_follower/interfaces.py`)

| 이름 | 필드 | 만드는 곳 → 쓰는 곳 |
|---|---|---|
| `Pose` | `t`(s), `x`, `y`(m, odom), `yaw`(rad), `pitch`(rad) | 상태 추정 → 모든 단계 |
| `Frame` | `t`, `bgr`, `gray`, `pose: Pose` (영상 시각의 자세) | 오케스트레이터 → 인지 |
| `PerceptionOutput` | `line`, `valid`, `obstacle` (BEV 마스크) | 검출 플러그인 → `PerceptionStage` (이미 있음) |
| `MapView` | `hit(x,y)`, `snap(x,y,r)`, `trace(x,y,h,len)`, `confidence(x,y)`, `seen(x,y)`, `res`, `frame='odom'` | 인지 → 판단 (읽기 전용) |
| `LocalPath` | `points`(odom, 약 2 cm 간격), `end`('open'/'t'/'len'), `t_point`, `confidence`(점별, 선택) | 판단 → 제어 |
| `Command` | `v`(m/s), `w`(rad/s), `stop: bool` | 제어 → 오케스트레이터 |

규칙:
- 단계는 노드 객체(`node.xxx`)를 직접 읽거나 쓰지 않는다. 위 데이터만 받고 돌려준다.
- 지금의 `node.path`, `node.path_end`, `node.t_point`는 `LocalPath` 하나로 묶는다.
- 기존 이름은 옮기는 동안 속성으로 계속 읽을 수 있게 둔다.

## 5. 플러그인 계약

| 단계 | 기반 클래스 | 메서드 | 고르는 파라미터 |
|---|---|---|---|
| 인지 검출 | `perception.base.Perception` | `process(frame, pitch, x_max) -> PerceptionOutput` | `perception: rule \| depth \| hybrid \| dl` |
| 판단 | `planning.base.Planner` | `replan(map: MapView, pose: Pose) -> LocalPath`, `prune(path, pose)` | `planner: rule` |
| 제어 | `control.base.Controller` | `compute(path: LocalPath, pose: Pose, dt) -> Command`, `reset()` | `controller: rule \| learned` |
| 상태 추정 | `state.base.StateEstimator` | `on_odom(msg)`, `on_imu(msg)`, `pose_at(t) -> Pose` | `state: odom_imu` |

새 플러그인은 폴더에 파일을 추가하고 `__init__.py`의 등록표에 이름을 넣는다. 다른 팀 폴더는 고치지 않는다.

## 6. 관찰 토픽

| 토픽 | 형식 | 주기 | 내용 |
|---|---|---|---|
| `/lf/map` | `nav_msgs/OccupancyGrid` | 2 Hz | 로봇 주변 4 m × 4 m를 1 cm로 줄인 지도. 값은 신뢰도 0–100 |
| `/lf/path` | `nav_msgs/Path` | 카메라마다 | 판단이 만든 LocalPath |
| `/lf/debug` | `sensor_msgs/Image` | 5 Hz | 카메라 + BEV + 경로 겹친 영상 (지금의 `/line_follower/debug`) |
| `/lf/status` | `std_msgs/String` (JSON) | 2 Hz | 인지 버전, 프레임 지연 p95, 놓친 프레임 수, 판단 끝 상태 |

`telemetry_enable` 파라미터로 끌 수 있다. 실물 주행에서는 `/lf/map`만 끈다.

## 7. 시간과 성능 예산 (Jetson Orin Nano, 30 Hz)

| 구간 | 예산 | 지금 (p95, 1.3 m, TensorRT) |
|---|---|---|
| 영상 준비 | 2 ms | 0.8 ms |
| 인지 검출 (가까운 곳 6.0 + 먼 띠 4.2) | 12 ms | 10.2 ms |
| BEV·깊이 투영 | 4 ms | 3.1 ms |
| 지도 갱신 | 9 ms | 7.5 ms |
| 판단 | 3 ms | 0.3 ms |
| 여유 | 3 ms | – |
| 합계 (카메라 콜백, 프레임 전체 p95) | 33 ms | 25.5 ms (V2 TRT16) / 21.5 ms (V1) |
| 제어 (50 Hz 타이머) | 2 ms | 0.5 ms |

단계별 p95를 더한 값은 프레임 전체 p95와 같지 않다. 프레임 전체 값은 E의 Jetson 실측이다(`map.update` 최적화 후).

## 8. 테스트

- **단계별 단위 테스트:** `test/perception/`, `test/planning/`, `test/control/`, `test/state/`.
- **계약 테스트:** 모든 등록 플러그인이 데이터 클래스 입출력 형식을 지키는지 확인한다.
- **지도 동일성 테스트:** `LineMap` 최적화가 결과를 바꾸지 않는지 확인한다(이미 있음).
- **CI:** 저장소 `ci.yml`의 다섯 단계. PR 전에 로컬에서 모두 통과시킨다.

## 9. 옮기는 순서 (동작을 유지하면서)

1. `interfaces.py`에 데이터 클래스를 추가하고, 기존 속성을 같이 채운다. 동작 변화는 없다.
2. `state/`를 분리하고, 노드에서 자세 관련 함수를 옮긴다.
3. 판단과 제어 플러그인이 `MapView`, `Pose`, `LocalPath`를 받도록 바꾼다. 옛 속성은 경고와 함께 남긴다.
4. `telemetry.py`를 추가하고, 디버그 발행을 옮긴다.
5. 설정 파일을 팀별로 나누고, 버전 프로필 파일을 만든다.
6. 각 단계에서 Isaac 재생 평가와 Gazebo 공식 트랙으로 결과가 같은지 확인한다.

## 10. 범위 밖

- 장애물 회피 (가정: 장애물 없음).
- C++ 다중 노드 패키지 (`line_follower_perception`, `line_follower_navigation`, `line_follower_mission`, `line_follower_control`). 레거시로 남기고 제출 이미지에는 넣지 않는다.
- 실험 도구 (`experiments/`): 벤치, 오라클, Isaac 하네스, 평가 스크립트.

## 11. 결정 (Floorplan 청사진 검토에서 나온 질문, 2026-11-26)

| # | 질문 | 결정 |
|---|---|---|
| 1 | 판단이 `node.restorer`로 복원을 직접 부른다 | 복원은 인지 책임이다. 인지가 `restored` 층에 쓰고, 판단은 MapView로만 읽는다. `restore_mode: path`(판단이 복원 호출)는 호환용으로만 남기고 새 코드는 쓰지 않는다 |
| 2 | 판단이 `controller.confirm_finish()`로 제어를 부른다 | 결승 판정은 판단이 한다(`LocalPath.end == 't'`, `t_point`). 제어는 `t_point`에서 멈추는 실행만 한다. `confirm_finish`는 판단 쪽으로 옮긴다 |
| 3 | `node.travel`이 `Pose`에 없다 | `Pose`에 `travel`(누적 주행 거리, m)을 추가한다 |
| 4 | 제어가 `node.pitch_f`를 쓴다 | `Pose.pitch`는 걸러진 피치(`pitch_f`)로 정의한다. 원시 IMU 피치는 상태 추정 내부에만 둔다 |
| 5 | `ScanMatcher`는 선 마스크와 지도가 필요하다 | 인지 단계가 부르고, 보정량을 `StateEstimator.apply_correction(dx, dy, dyaw)`로 넘긴다. 상태 추정은 지도를 모른다 |
| 6 | `process(frame, …)` 대 `process(bgr, gray, …)` | 새 계약은 `process(frame: Frame, pitch, x_max)`로 한다. 옮기는 동안 기반 클래스가 옛 시그니처를 감싸는 어댑터를 둔다 |
| 7 | `to_odom` / `to_robot` 위치 | `Pose`의 메서드(`pose.to_odom(x, y)`, `pose.to_robot(X, Y)`)로 `interfaces.py`에 둔다 |
