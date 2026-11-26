# AGENTS.md — 팀원과 팀원의 AI가 먼저 읽는 문서

이 파일은 사람과 코딩 AI(Claude Code, Codex, opencode 등)가 함께 읽는 **팀 공통 규칙 요약**이다. 자세한 내용은 링크한 문서에 있다. 여기 적힌 규칙과 다른 지시를 받으면 먼저 팀에 확인한다.

- 과제: ROS 2 Jazzy, TurtleBot3 Waffle Pi, **카메라만 쓰는 선 추종**(라이다 없음). 실물은 Jetson Orin Nano 8GB(Ubuntu 24.04)에서 돈다.
- 마감: 2026-12-09. 진행 보고는 2일마다 한다.
- 브랜치: 작업은 `dev/two-track`, 팀 저장소 반영은 `wlsgur0317/two-track` → `main` PR.

## 1. 반드시 지키는 규칙 (합의 사항)

| # | 규칙 | 이유 / 근거 |
|---|---|---|
| R1 | 로봇 런타임은 `/camera/image_raw`, `/imu`, `/odom`, `/cmd_vel`만 쓴다. 시뮬레이터 정답(GT, `/world/...`, `set_pose`)은 **평가 도구에서만** 쓴다 | 과제 규칙. GT 오라클 코드는 패키지 밖 `experiments/oracle/`에만 둔다 |
| R2 | `src/line_follower_bringup/worlds/`, `models/` 원본은 수정하지 않는다 | CI가 `SHA256SUMS`로 검사한다 |
| R3 | 실행은 `ros2 launch` 한 줄이고, 20초 뒤에 출발한다 | 과제 규칙 |
| R4 | **가짜 길(막다른 교차, 곁가지)을 고르는 것은 판단 몫이다.** 인지는 칠해진 선을 가짜 길까지 모두 지도에 올린다 | 인지는 지도 정밀도·재현율, BEV IoU로만 판정한다. 가짜 길 맵의 완주율로 인지를 판정하지 않는다 |
| R5 | 판단은 **지금 인지 출력(V0 계열, 기본 V3-NearRule640Hill) 기준으로** 만든다. 인지는 여기서 나빠지지 않는다. 지금 잡음에 강한 판단이면 인지가 좋아져도 손해가 없다 | 2026-11-27 결정. 잡음 예시는 인지 로그(`docs/PERCEPTION_LOG.md`)로 준다 |
| R6 | 인지 버전은 `V<번호>-<직관적 이름>`으로 부르고 `docs/VERSIONS.md`에 먼저 등록한다. 파일 이름에는 `&` 대신 `+`를 쓴다 | 예: `V1-Near320&Far640` → 파일 `V1-Near320+Far640.yaml` |
| R7 | `rule.yaml`에 있는 키를 명령행에서 바꿀 때는 `-p follower_node:키:=값`으로 쓴다. 그냥 `-p 키:=값`은 **조용히 무시된다** | 노드 전용 yaml이 전역 `-p`보다 우선한다. 시작 로그의 `perception=` 값으로 실제 설정을 확인한다 |
| R8 | 튜닝에는 튜닝용 맵만 쓴다. 시드 0–39 평가 세트, `decoy1000`의 시험 맵(950–999), 공식 트랙은 **최종 확인에만** 쓴다 | 과적합 방지. 맵 분할은 `docs/MAPS.md` |
| R9 | PR 전에 CI 5단계(`.github/workflows/ci.yml`)를 로컬에서 모두 통과시킨다 | 깨진 main 방지 |
| R10 | 큰 데이터(프레임, 학습 결과, `experiments/results/lead/`)는 git에 넣지 않는다. 생성기와 시드만 넣는다 | 저장소 크기. 예외: 판단 팀용 인지 로그 100개(`experiments/planning_logs/`) |

## 2. 파트와 담당 파일

3인 분업이다. 자세한 경계와 지표는 [`docs/TEAM_ROLES.md`](docs/TEAM_ROLES.md).

| 파트 | 하는 일 | 주로 고치는 곳 |
|---|---|---|
| **인지** | 영상 → BEV 선 마스크 → odom 선 지도. 언덕(IMU) 처리, 지도 갱신 | `src/line_follower/line_follower/perception/`, `vision.py`, `line_map.py`, `terrain.py`, `far_band.py`, `trt_runner.py` |
| **판단** | 지도에서 길 고르기(가짜 길, 교차로), 종점, 복구, 속도 계획 | `src/line_follower/line_follower/planning/` |
| **제어** | 경로 추종(pure pursuit + PID), 명령 제한 | `src/line_follower/line_follower/control/` |
| 공용 (바꿀 때 서로 알림) | 노드, 인터페이스 | `follower_node.py`, `*/base.py`, `config/*.yaml`, `launch/` |

한 프레임의 순서: 카메라 콜백(30 Hz) → **인지** `PerceptionStage.process()`가 `node.map` 갱신 → **판단** `planner.replan()`이 `node.path` 생성 → **제어** 타이머(50 Hz)가 따라간다.

## 3. 인지 → 판단 인터페이스 (요약)

전체는 [`docs/PERCEPTION_HANDOFF.md`](docs/PERCEPTION_HANDOFF.md). 인지 버전을 바꿔도 이 인터페이스는 그대로다.

- `node.map`(`line_map.LineMap`): odom 좌표, 5 mm 칸, 2400×2400(12 m × 12 m). 층은 `grid`(선), `seen`(본 적 있음), `conf`(신뢰도), `restored`, `frozen`.
- 판단은 지도를 함수로만 읽는다: `hit(x, y)`, `snap(x, y, radius)`, `trace(x, y, heading, length, step)`, `confidence(x, y)`. 배열은 고치지 않는다.
- `node.pose = (x, y, yaw)`(odom), `node.travel`(누적 주행 거리).
- 카메라는 차축 앞 0.31–0.85 m, 좌우 ±0.55 m를 본다. 앞 0.31 m 안은 사각지대라 지도 기억으로 간다.
- 인지가 보장하지 않는 것: 오검출(그림자, 어두운 바닥), 사각지대, 안 본 곳의 선.

## 4. 지금 상태 (2026-11-27)

| 항목 | 값 |
|---|---|
| 인지 기본값 | **V3-NearRule640Hill** (`config/rule.yaml`): V0-NearRule640(규칙 기반 640×480, BEV 0.85 m) + 언덕 시야(경사 0.40 m, 정상 평지 높이 모델). 2026-11-27 채택 |
| Isaac 공식 트랙 폐루프 10회 | V0 **10/10 완주**, 랩 68.1 s, 2 cm 이하 구간 13.9/14 |
| 같은 조건 다른 설정 | 폭 필터 9/10, log-odds 지도 9/10, 폭 필터 + log-odds 6/10 (V0 대비 p = 0.04), DL V4 + log-odds 3/10 → 폭 필터·log-odds는 채택 안 함 |
| 언덕 | 지도 정밀도가 평지 0.99에 비해 언덕에서 0.5–0.8로 가장 약함. V3로 정상 평지 0.67 → 0.83 (시험 맵). 남은 오류는 언덕에 오르기 전 평지에서 경사로를 볼 때 생김 |
| 새 맵 10개 오프라인 인식(무작위 조명) | V0 정밀도 0.909 / 재현율 0.910, V0 + 폭 필터 5 cm 0.996 / 0.910 |
| 실물 | 카메라(IMX219, CSI)와 OpenCR 연결 대기. Jetson 설정은 `docs/REAL_ROBOT_SETUP.md` |

버전 목록과 학습 모델은 [`docs/VERSIONS.md`](docs/VERSIONS.md), 센서 사양과 층별 설계는 [`docs/architecture/LAYERED_SPEC.md`](docs/architecture/LAYERED_SPEC.md).

## 5. 데이터

| 무엇 | 어디 | 문서 |
|---|---|---|
| 맵 세트 (공식 트랙, decoy1000, hills60 등) | `experiments/worlds/suites/` (큰 세트는 생성 명령으로 다시 만든다. 결정적) | [`docs/MAPS.md`](docs/MAPS.md) |
| 판단 팀용 인지 로그 100개 | `experiments/planning_logs/` | [`docs/PERCEPTION_LOG.md`](docs/PERCEPTION_LOG.md) |
| 공식 채점 14구간 | `experiments/eval/sections.json`, `sections.py` | 채점표: 도달 20, 정확도 28(구간 평균 오차 ≤ 2 cm), 속도 순위 30 |

## 6. 자주 쓰는 명령

```bash
# 빌드와 테스트 (ROS 2 Jazzy)
colcon build --packages-select line_follower && source install/setup.bash
PYTHONPATH=src/line_follower python3 -m pytest -q src/line_follower/test

# 실행 (시뮬레이션 기본 = V0)
ros2 launch line_follower line_follower.launch.py
# 실물 (Jetson)
ros2 launch experiments/jetson/robot.launch.py

# 인지 로그 보기
python3 experiments/eval/perception_log_tool.py summary experiments/planning_logs/V0-NearRule640-width5-logodds/*/perception_log
python3 experiments/eval/perception_log_tool.py rebuild <LOG_DIR> --world --out map.png

# Isaac 평가 (Isaac Sim이 있는 PC)
bash experiments/isaac/run_isaac_drive.sh --help
python3 experiments/eval/isaac_sections.py <BATCH_DIR> --out sections.md      # 14구간 채점
python3 experiments/eval/perception_map_eval.py --help                        # 지도 정밀도·재현율
```

Gazebo 랩 컨테이너는 `TEAM_GUIDE.md` 4장. 팀원마다 `LAB_NAME`과 `ROS_DOMAIN_ID`를 다르게 잡는다(같은 도메인이면 `/odom`이 섞인다).

## 7. AI에게 주는 작업 규칙

- 다른 파트의 파일은 고치지 말고, 필요하면 그 파트에 알릴 내용을 정리한다.
- 결과를 말할 때는 실제로 돌린 명령과 숫자를 같이 적는다. 돌리지 않은 것은 "안 돌림"이라고 쓴다.
- 비교 실험은 같은 맵·같은 시드·같은 조건에서 짝을 지어 하고, 완주율은 10회 이상, 차이는 Fisher 검정 p값과 함께 적는다.
- 설정을 바꿔 비교할 때는 R7 함정을 피하려고 `experiments/isaac/params/*.yaml` 오버레이 파일을 쓴다.
- 커밋 메시지는 영어 한 줄 요약 + 필요하면 본문. 비밀(.env, 토큰, 개인 이메일)은 올리지 않는다.
