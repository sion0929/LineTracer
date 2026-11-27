# 노드별 개발 안내 (인지 · 지도 · 판단 · 제어)

2026-11-27 · 사람과 AI가 같이 읽는 문서. 설계 배경은 `docs/architecture/NODE_SPLIT.md`.

각 팀은 **자기 노드만** 만든다. 다른 팀 노드는 지금 코드를 옮긴 더미가 대신 돌아서, 혼자서도 전체 주행을 시험할 수 있다. 토픽 계약(아래 2장)만 지키면 언어(Python / C++)도 자유다.

## 1. 노드와 담당

| 노드 | 담당 | 지금 들어 있는 것 | 실행 이름 |
|---|---|---|---|
| 인지 `perception_node` | 인지 | 상태 추정(odom + IMU, 언덕 보정) + 선 검출 + BEV 투영 + 언덕 시야(V3) | `ros2 run line_follower perception_node` |
| 지도 `map_node` | 인지 | `LineMap` 누적, 패치 발행 | `ros2 run line_follower map_node` |
| 판단 `planning_node` | **판단 팀** | 더미 = 지금 `RulePlanner` | `ros2 run line_follower planning_node` |
| 제어 `control_node` | **제어 팀** | 더미 = 지금 pure pursuit + PID, 결승 정지, 길 잃으면 탐색 | `ros2 run line_follower control_node` |

## 2. 토픽 계약 (바꾸려면 세 팀이 합의)

| 토픽 | 메시지 | 보내는 쪽 → 받는 쪽 | 주기 | 내용 |
|---|---|---|---|---|
| `/lf/perception/frame` | `PerceptionFrame` | 인지 → 지도 | 카메라마다 (30 Hz) | 촬영 시각·자세, 언덕 상태, BEV 선/유효 마스크 |
| `/lf/pose` | `LfPose` | 인지 → 모두 | odom마다 | odom 자세 `x, y, yaw`, 누적 거리 `travel`, 피치, 높이, 언덕 상태 `slope` |
| `/lf/map/patch` | `LineMapPatch` | 지도 → 판단 | 프레임마다 + 1초마다 전체 | 이번 프레임에 바뀐 지도 사각형(선 `grid`, 본 곳 `seen`, 신뢰도 `conf`) |
| `/lf/map` | `nav_msgs/OccupancyGrid` | 지도 → rviz | 2 Hz | 로봇 주변 4 m × 4 m, 1 cm (보기용) |
| `/lf/path` | `LinePath` | 판단 → 제어 | 패치마다 | odom 점 목록(가까운 점부터 약 2 cm), 끝 상태 `open`/`t`/`len`, 결승 T 위치 |
| `/cmd_vel` | `geometry_msgs/Twist` | 제어 → 로봇 | 50 Hz | 속도 명령 |

메시지 정의: `src/line_follower_interfaces/msg/`. 좌표계는 전부 **odom**(출발점 원점, x = 출발 방향, y = 왼쪽), 지도 칸은 5 mm, 칸 `(i, j)`는 `y = (i - 1200) × 0.005`, `x = (j - 1200) × 0.005`.

## 3. 내 노드로 바꿔 끼우기

```bash
# 더미 판단을 빼고 띄운 뒤, 내 판단 노드를 따로 띄운다
ros2 launch line_follower split.launch.py planning:=none
ros2 run <내 패키지> <내 판단 노드>      # /lf/map/patch + /lf/pose 받고 /lf/path 보냄

# 제어도 같은 방법
ros2 launch line_follower split.launch.py control:=none
```

- 파라미터: launch가 `config/rule.yaml`의 `follower_node:` 절을 네 노드 모두에 준다. 내 노드 파라미터는 내 yaml로 따로 준다.
- 출발 20초 지연 규칙은 launch가 지킨다(`start_delay`).
- 지도 사본 만들기(판단, Python): `line_follower.split.codec.paste_patch(line_map, patch)`로 패치를 붙이면 지도 노드와 칸 하나까지 같은 `LineMap`이 생긴다. `LineMap`의 `hit / snap / trace / confidence`를 그대로 쓸 수 있다. 예시는 `line_follower/split/planning_node.py`.
- C++ 노드도 된다. 메시지를 구독해 `grid`(행 우선, `rows × cols` 바이트)를 자기 배열에 붙이면 된다.

## 4. 혼자 시험하는 법

| 팀 | 방법 |
|---|---|
| 판단 | 인지 로그(`experiments/planning_logs/`, `docs/PERCEPTION_LOG.md`)를 `LineMap`으로 다시 쌓아 내 판단을 주행 없이 돌린다(`experiments/eval/log_filter_eval.py`의 `rebuild` 참고). 정답 경로는 맵 JSON의 `centerline` |
| 제어 | `/lf/path`를 녹화한 bag이나 직선·원호 경로를 직접 발행해 추종 오차를 잰다 |
| 공통 | Isaac: `bash experiments/isaac/run_isaac_drive.sh --run-split --map official --out <폴더>` (먼저 컨테이너 `/ws`에서 `colcon build --packages-up-to line_follower`), 채점은 `python3 experiments/eval/isaac_sections.py <배치 폴더>` |

## 5. 처리 시간

노드마다 10초에 한 번 콜백 처리 시간 p50 / p95를 로그로 남긴다(`[timing] ...`). 30 Hz 예산은 프레임당 33 ms이고 Jetson에서 인지 + 지도가 대부분을 쓴다. 무거운 단계는 측정 뒤 C++로 옮길 수 있다(`NODE_SPLIT.md` 4장).

## 6. 지켜야 할 것

- `AGENTS.md` 규칙(R1–R10)을 따른다. 특히 런타임은 `/camera/image_raw`, `/imu`, `/odom`, `/cmd_vel`만 쓴다.
- 다른 팀 노드 파일은 고치지 않는다. 계약(2장)을 바꿔야 하면 이슈나 PR 설명에 적고 합의한다.
- 기존 한 노드 실행(`ros2 launch line_follower line_follower.launch.py`)도 그대로 돈다. 두 방식은 같은 입력에서 같은 지도·경로를 만든다(`src/line_follower/test/test_split_nodes.py`).
