# ROS 2 Line Follower Rover

[![CI](https://github.com/sion0929/LineTracer/actions/workflows/ci.yml/badge.svg)](https://github.com/sion0929/LineTracer/actions/workflows/ci.yml)

TurtleBot3 Waffle Pi 라인 추종 로버. ROS 2 Jazzy + Gazebo Harmonic. 설계서: [`docs/design.md`](docs/design.md) (v1.2).

## 1. 프로젝트 개요

Robot Programming 개인 프로젝트 #1 과제로, 공식 PDF가 요구하는 조건을 그대로 따른다.

| 과제 조건 | 수용 방식 |
|---|---|
| 한 줄 `ros2 launch` 실행 | `ros2 launch line_follower_bringup competition.launch.py` 하나로 Gazebo·브리지·인식·판단·제어·안전·미션 노드가 모두 기동 |
| 20초 대기 후 주행 | `mission_manager`가 기동 후 20초 + 센서/브리지 준비 확인 전까지 속도 0 유지, 이후 자동 시작 |
| 제공 월드·로버 모델 변경 금지 | `src/line_follower_bringup/worlds`, `models`는 원본 그대로 보존, `SHA256SUMS`로 해시 검증 |
| 로버 카메라 영상 기반 인식 | 입력은 `/camera/image_raw` 한 가지뿐, C++ OpenCV로 라인 검출 |
| 로버 토픽만 사용 | `/camera/image_raw`, `/imu`, `/odom`, `/cmd_vel`만 사용. `/world/` 접두 토픽·`set_pose` 등 시뮬레이터 실제 위치 인터페이스는 코드 전역에서 정책 테스트로 차단 |

과제 제출물은 패키지 압축 파일, Docker 이미지 주소, 실행 명령어 한 줄이며 기한은 2026-11-22(일) 23:59다.

## 2. 실행 방법

### 2.1 Docker로 실행 (권장, 평가 방식과 동일)

이 저장소 루트(`Dockerfile`이 있는 위치)에서 이미지를 빌드한다.

```bash
docker build -t line_follower:latest .
```

GUI(X11)로 실행:

```bash
xhost +local:docker
docker run --rm -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix line_follower:latest
```

컨테이너의 기본 명령이 바로 공식 한 줄 실행이다.

```bash
ros2 launch line_follower_bringup competition.launch.py
```

Headless(화면 없이) 실행:

```bash
docker run --rm -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix line_follower:latest \
  ros2 launch line_follower_bringup competition.launch.py headless:=true
```

### 2.2 네이티브 colcon 빌드 (ROS 2 Jazzy + Gazebo Harmonic 환경에 직접 설치된 경우)

```bash
colcon build --packages-up-to line_follower_bringup
source install/setup.bash
ros2 launch line_follower_bringup competition.launch.py
```

### 2.3 개발용 실행 — `dev_sim` + GCS

평가용 `competition` 프로파일은 GCS 없이 완전 자율로 동작하지만, 개발 중에는 아래처럼 로컬 맵·카메라·명령 패널을 보며 디버깅한다. 과제용 수업 이미지 `rp/ros2:jazzy-lab` 기반 랩 컨테이너를 사용한다(`tools/lab_env.sh`).

```bash
tools/lab_env.sh build
tools/lab_env.sh gcs headless:=false auto_start:=false
```

GCS 창이 뜬 뒤 20초 센서 게이트가 지나면 **GCS의 "미션 시작" 버튼**으로 주행을 시작한다(`auto_start:=false`이므로 자동 시작하지 않음). 종료는 다른 터미널에서 `tools/lab_env.sh gcs-stop` 또는 창을 닫거나 Ctrl-C.

### 2.4 검증 명령

아래 명령은 모두 랩 컨테이너(`tools/lab_env.sh`) 안에서 실제 Gazebo를 띄워 동작을 검사한다. CI에서는 실행하지 않는다(§4 참고).

| 명령 | 확인 내용 |
|---|---|
| `tools/lab_env.sh test` | colcon 단위/launch 테스트 전체 |
| `tools/lab_env.sh checkpoint-a` | D-01~D-04: 5노드 그래프, 센서, 기본 구동, 20초 게이트·안전 정지 |
| `tools/lab_env.sh check-sensors` / `check-motion` / `check-start-stop` / `check-guard-exit` | Checkpoint A 개별 항목 |
| `tools/lab_env.sh check-d05` / `check-d06` / `check-d07` | 라인 검출, 로컬 경로 기억, 사각지대·언덕 유효성 |
| `tools/lab_env.sh check-drive` | 교차·코너·커브·언덕 구간 및 전체 트랙 자율 주행 시험 |
| `tools/lab_env.sh check-gcs` / `check-bag` | 개발용 GCS 통신·UI, ROS Bag 기록/재생 |

## 3. 설계 요약

### 3.1 노드 구조와 토픽 흐름

기본 실행 그래프는 **컴포저블 노드 하나(perception_container)** + 독립 프로세스 3개로 구성된다.

```text
perception_container (component_container_mt, 실행기 스레드 4개)
  camera_bridge [ros_gz_bridge] ─► /camera/image_raw ─► line_detector ─► /perception/line ─► path_planner ─► /path/local
                                                                                                    │
/imu, /odom (별도 브리지 프로세스) ────────────────────────────────────────────────────────────────┤
                                                                                                    ▼
                                                                                          motion_controller
                                                                                    (Pure Pursuit + 회전 PID)
                                                                                                    │  /control/cmd_vel_candidate
mission_manager (20초 대기·상태·완주 판정) ──► /mission/state ──────────────────────────────────►  ▼
                                                                                          safety_guard (유일한 /cmd_vel 발행) ──► Gazebo
```

| 구성 요소 | 역할 |
|---|---|
| `line_detector` (perception_container 컴포넌트) | 카메라 브리지 입력 → C++ OpenCV 라인 검출 → 중심선·교차/코너 그래프 → `/perception/line` |
| `path_planner` (perception_container 컴포넌트) | Local Path Mapper/Memory, 후보 평가, 경로·속도·회전 모드 결정 → `/path/local` |
| `motion_controller` | Pure Pursuit + 회전 PID로 속도 후보 산출 (`/cmd_vel` 직접 발행 금지) |
| `safety_guard` | 주행 허가·신선도·속도/가속 검사 후 **유일하게 `/cmd_vel`을 발행** |
| `mission_manager` | 20초 대기, 준비 검사, 미션 상태 머신, 완주 판정 |

### 3.2 라인 검출 (OpenCV)

640×480 카메라 입력을 C++ OpenCV로 처리한다: Otsu 임계값 + 국소 대비 보정, 4mm 지면 격자 투영, 위상 보존 세선화, 교차/코너 분기를 명시적으로 분리한다. 촬영 시각(`capture_stamp`)은 그대로 유지되어 이후 오도메트리 매칭에 사용된다. Fast-SCNN 기반 딥러닝 백엔드는 설계상 후보(스텁 + OpenCV 폴백)로 남아 있다.

### 3.3 로컬 경로 기억

- 카메라가 **실제로 관측한 선분만** `odom` 좌표계로 짧게 저장한다(최대 1,200점/32선분, 기본 30초 만료).
- 촬영 시각의 로봇 자세(오도메트리 보간 + 자이로 헤딩)를 선분과 함께 저장해, 선이 일시적으로 사각지대에 들어가도 저장된 선분을 현재 위치로 투영해 추종한다(`MEMORY_TRACKING`).
- 다시 보이면 기억과 비교해 일치할 때만 `VISION_TRACKING`으로 복귀(`REACQUIRING`).
- 언덕 구간은 IMU 피치로 지면 투영 유효성을 판정하고, 투영이 무효하면 **영상좌표 기반 추종**(`IMAGE_TRACKING`, 최대 0.06 m/s)으로 전환한다.

### 3.4 교차로 판단

`line_detector`는 교차/분기 후보와 주변 연결 관계만 보고하고, 방향 선택은 하지 않는다. `path_planner`가 접근 중 관측 가능한 후보를 최대한 모으고, 진입 헤딩과의 연속성·관측 길이·곡률로 점수를 매겨(규칙 기반, MLP는 설계상 보조 후보로 남아 있으며 현재 비활성) 경로를 선택한 뒤 **잠금**한다. 새 관측이 반박하거나 안전 문제가 있을 때만 재계획한다.

### 3.5 제어

- 기본 추종: Pure Pursuit.
- 급격한 방향 전환: 먼저 완전 정지 후 회전 PID로 목표 헤딩 정렬(제자리 회전에 시간 상한 존재, 초과 시 해당 에피소드 속도 0).
- `safety_guard`가 신선도·속도·가속도 한계를 검사한 뒤 최종 명령만 통과시킨다.

### 3.6 미션·안전

- 기동 후 20초 + 센서/브리지 준비 확인 전까지 속도 0 (R-04).
- GCS의 `KILL`은 소프트웨어 정지 요청(E_STOP→STOPPED)이며 **물리적 E-STOP이 아니다**.
- 제어 입력(카메라/경로/오도메트리)이 끊기면 타임아웃 내 정지.
- 종료(SIGINT/SIGTERM) 시 실행기를 먼저 멈춘 뒤 0 명령을 반복 발행해 잔류 속도를 남기지 않는다.

### 3.7 개발용 GCS

노트북에서 실행하는 개발 전용 시각화/기록 도구로, 평가용 `competition` 프로파일은 이것 없이도 완전 자율 동작한다.

- **4×4 UI**: 좌측 3×3은 지도(관측=녹색, 기억=청록, 선택 경로=파랑, 분기 후보=노랑 점선), 우측 상단 명령 패널, 우측 하단 카메라, 맨 아래 ROS Bag 타임라인.
- **확정 경로 맵**: 로버의 `path_planner`가 보낸 관측/기억/선택 경로를 2Hz 스냅샷으로 받아 그릴 뿐, GCS가 경로를 생성하거나 전체 트랙을 미리 알고 그리지 않는다.
- **ROS Bag 기록/재생**: `gcs_recorder`가 MCAP(Zstd 압축)로 기록하고, 재생은 실제 구동 `/cmd_vel`에 연결하지 않는 별도 경로(`/replay/cmd_vel_blocked`)로 안전하게 분리된다.

### 3.8 검증 체계

설계서 §17 체크리스트(D-01~D-16)를 단계별 완료 게이트로 사용하며, 각 항목은 Gazebo 자동 검사(`tools/lab_env.sh check-*`)의 실측 결과로만 완료 처리한다. 코드/문서 전역 정책 테스트(`test_static_policy.py`)가 `/world/` 접두 토픽·`set_pose` 미사용과 `safety_guard` 단독 `/cmd_vel` 발행을 전 소스에서 강제한다.

## 4. 현재 상태 (정직하게)

**전체 트랙 완주는 아직 달성하지 못했다.** 공식 `competition.launch.py` 전체 주행 반복 시험에서 기록한 **최고 진행률은 76.8%(12.06 m / 15.70 m)** 이며, 같은 조건의 다른 반복에서는 훨씬 낮은 진행률(약 19~74%)도 나와 실행 간 편차가 크다. 현재까지 확인한 결함은 대부분 Ω자 루프 구간 진입부·교차 이후 재진입에서 라인 재획득 실패 또는 제자리 회전 복구 시간 초과로 멈추는 것이다.

개선 진행 중인 항목(설계서 §17 D-10, D-14, D-15):

- **D-10 목적지 도착 판정**: 출발 T자와 목적지 T자 구분, 로봇 중심 결승선 통과 판정 및 자동 종료 — 설계·단위 테스트는 작성되었으나 Gazebo 전체 주행 통과 확인이 아직 완료되지 않음.
- **속도 프로파일 개선**: 곡률·관측 거리·제동 여유 기반 속도 상한의 구간별 튜닝.
- **구간별 패턴 대처표 학습**: 교차 이후 재진입·Ω 루프 등 반복 실패 구간에 대한 회귀 테스트 보강과 완주 성능 개선.

단정적인 "완주 성공" 또는 "항상 안정적으로 동작" 같은 주장은 하지 않으며, 수치는 모두 `docs/d0809_evidence.md`, `docs/r3_evidence.md`, `docs/d10_evidence.md`의 실측 기록을 근거로 한다.

## 5. 폴더 구조

| 경로 | 내용 |
|---|---|
| `src/line_follower_interfaces` | 메시지/서비스 정의 (설계 §5.2) |
| `src/line_follower_perception` | `line_detector` 컴포넌트 — OpenCV 기준선 인식 + Fast-SCNN 스텁/폴백 |
| `src/line_follower_navigation` | `path_planner` 컴포넌트 — LocalPathMapper/Memory, PoseEstimator, CandidateFeatureBuilder, 규칙/MLP 평가, TrajectoryPlanner |
| `src/line_follower_control` | `motion_controller`(Pure Pursuit + 회전 PID), `safety_guard`(유일한 `/cmd_vel` 발행자) |
| `src/line_follower_mission` | `mission_manager` — 20초 게이트, 상태 머신, 완주 판정 |
| `src/line_follower_bringup` | `competition`/`dev_sim`/`replay` launch, 파라미터, 브리지 설정, 제공 `worlds/`+`models/` 원본(`SHA256SUMS`로 무결성 보장) |
| `src/line_follower_tests` | launch 통합 테스트, 정책 검사(`test_static_policy.py`), 자산 무결성 검사(`test_asset_integrity.py`) 등 |
| `tools/gcs` | 개발 전용 GCS(Qt), 명령 브로커, Bag 기록기 (`line_follower_gcs` 패키지) |
| `tools/bag_analyzer` | 개발 전용 rosbag 이벤트 분석 도구 |
| `tools/lab_env.sh`, `tools/check_*.py` | 랩 컨테이너 관리 및 Gazebo 기반 검증 스크립트 |
| `docker/`, `Dockerfile` | 평가용 Docker 이미지 |
| `.github/workflows/ci.yml` | GitHub Actions CI (정적 검사 + ROS 2 Jazzy 빌드/테스트 + 경쟁 패키지 빌드) |
| `docs/` | 설계·검증 문서 |

### 문서 링크

- [`docs/design.md`](docs/design.md) — 설계서 v1.2 (전체)
- [`docs/decision_rules.md`](docs/decision_rules.md) — 판단 노드(path_planner) 판단 테스트 규칙·판단 학습 규칙
- [`docs/line_detection_rules.md`](docs/line_detection_rules.md) — 라인 검출 성능 테스트·성능 개선 규칙
- [`docs/test_plan.md`](docs/test_plan.md) — 테스트 계획
- [`docs/deployment.md`](docs/deployment.md) — 배포 노트
- [`docs/checkpoint_a_evidence.md`](docs/checkpoint_a_evidence.md), [`docs/d05_evidence.md`](docs/d05_evidence.md), [`docs/d06_evidence.md`](docs/d06_evidence.md), [`docs/d07_evidence.md`](docs/d07_evidence.md), [`docs/d0809_evidence.md`](docs/d0809_evidence.md), [`docs/r3_evidence.md`](docs/r3_evidence.md) — 단계별 검증 기록 (D-10 기록 `docs/d10_evidence.md`는 `wip/d10-finish` 브랜치)
- [`docs/gcs_stage1_evidence.md`](docs/gcs_stage1_evidence.md), [`docs/gcs_stage2_evidence.md`](docs/gcs_stage2_evidence.md) — GCS/ROS Bag 검증 기록
- [`docs/components_evidence.md`](docs/components_evidence.md) — 컴포저블 노드 성능 측정
- [`실행방법_README.md`](실행방법_README.md) — 제출용 실행 방법 요약(국문)

### 작업 브랜치 (인수인계 시점, 미병합)

| 브랜치 | 내용 | 상태 |
|---|---|---|
| `wip/d10-finish` | D-10 도착 T 판정·자동 정지, `check-d10` | 도착 구간에서 결승선 +0.003 m 정지(작성자 보고), 독립 검증 전 |
| `wip/r4-speed-record` | 직선/곡선 속도 계획, 지나간 경로 기록·빈 구간 추정 실험 | 기준선 대비 이득 확인 안 됨 |
| `wip/physics-check` | Gazebo 실제 자세 기반 물리 이상 감시 `check-physics` | 헤드리스 3회 이상 없음, 폭주 미재현 |
