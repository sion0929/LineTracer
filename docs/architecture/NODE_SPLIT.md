# 노드 분리 설계: 인지 · 지도 · 판단(더미) · 제어(더미)

2026-11-27 · 인지 파트

지금은 `follower_node` 하나가 인지 → 지도 → 판단 → 제어를 한 프로세스에서 돈다. 이것을 ROS 노드 4개로 나눈다. 판단과 제어는 **지금 코드를 그대로 옮긴 더미**로 고정하고, 인지 팀은 인지·지도 노드만 고친다. 나중에 판단·제어 팀이 자기 노드로 바꿔 끼운다. 기존 단일 노드(`follower_node`)와 launch는 그대로 남긴다.

## 1. 노드와 토픽

```
/camera/image_raw  /imu  /odom
        │
[perception_node]  인지 (상태 추정 + PerceptionStage)
        │  /lf/perception/frame   PerceptionFrame   (카메라 프레임마다, 30 Hz)
        │  /lf/pose               LfPose            (odom마다)
[map_node]         지도 (LineMap, 언덕 위치 기억은 인지가 이미 거른 마스크로 반영됨)
        │  /lf/map/patch          LineMapPatch      (프레임마다, 바뀐 사각형만)
        │  /lf/map                nav_msgs/OccupancyGrid (2 Hz, 로봇 주변 4 m × 4 m, 1 cm, rviz용)
[planning_node]    판단 더미 = 지금 RulePlanner (지도 사본에 패치를 붙여 같은 지도를 유지)
        │  /lf/path               LinePath          (패치마다)
[control_node]     제어 더미 = 지금 follower_node.control + RuleController (50 Hz)
        │
     /cmd_vel
```

## 2. 메시지 (`line_follower_interfaces/msg`)

| 메시지 | 필드 |
|---|---|
| `PerceptionFrame` | `header`(stamp = 촬영 시각, frame_id = odom), `bool has_pose`, `float64 x, y, yaw`(촬영 시각 자세), `float32 pitch, imu_pitch, elev, travel`, `uint8 slope`(0 평지 1 오르막 2 정상 평지 3 내리막), `bool use_for_map`, `bool freeze`, `float32 freeze_radius`, `uint16 rows, cols`, `float32 res, x_near, x_far, y_half`, `uint8[] line, valid, far, add_only`(np.packbits, 없으면 빈 배열) |
| `LineMapPatch` | `header`, `int32 i0, j0`(2400 격자에서 왼쪽 위 칸), `uint16 rows, cols`, `uint8[] grid, seen, conf, frozen, restored`(없으면 빈 배열), `bool full`(처음 연결용 전체 창) |
| `LinePath` | `header`(frame_id = odom), `geometry_msgs/Point[] points`(가까운 점부터, 약 2 cm 간격), `string end_state`('open' / 't' / 'len'), `bool has_t_point`, `geometry_msgs/Point t_point` |
| `LfPose` | `header`, `float64 x, y, yaw`(odom), `float32 travel, pitch_f, elev`, `bool rocking`, `uint8 slope` |

## 3. 규칙

- 지도 칸 인덱스, 좌표계, BEV 격자는 지금과 같다(`docs/PERCEPTION_HANDOFF.md` 2장).
- 단일 노드와 분리 노드는 같은 입력에서 **같은 지도·같은 경로**를 만들어야 한다(단위 시험으로 확인).
- 파라미터: 네 노드가 지금 `follower_node`의 파라미터를 같이 선언한다. launch가 `rule.yaml`의 `follower_node:` 절을 네 노드 모두에 적용한다.
- 노드마다 콜백 처리 시간 p50 / p95를 10초마다 로그로 남긴다(언어 변경 판단용).
- 분리 모드에서 아직 지원하지 않는 옵션(`scan_match`, 복원 `restore_mode: map`)은 켜면 경고하고 무시한다. 둘 다 기본 꺼짐이다.

## 4. 언어 (C++ / 러스트) 판단 순서

1. 분리 노드에서 노드별 처리 시간과 CPU를 PC와 Jetson에서 잰다.
2. 가장 무거운 단계(예상: 지도 갱신, 규칙 기반 BEV)를 C++로 옮긴 판을 만들어 같은 입력으로 비교한다.
3. 30 Hz 예산(33 ms)과 Jetson CPU 여유를 기준으로, 차이가 크면 옮긴다. 러스트는 ROS 2 Jazzy 지원(rclrs)이 실험 단계라 C++을 먼저 본다. sion의 기존 노드와 OpenCV도 C++이다.

## 5. 구현 (2026-11-27)

| 무엇 | 어디 |
|---|---|
| 공용 코드 (파라미터 선언, 상태 추정, 제어 틱, 처리 시간 로그) | `line_follower/node_core.py` — `follower_node`와 분리 노드가 같이 쓴다 |
| 페이로드 (마스크 packbits, 프레임 적용, 패치, 미러 붙이기) | `line_follower/split/codec.py` (ROS 없이 시험 가능) |
| 노드 | `line_follower/split/{perception,map,planning,control}_node.py`, 실행 이름 `perception_node` 등 |
| launch | `ros2 launch line_follower split.launch.py [mode:=rule\|dl] [params_file:=...] [sim:=false]` — yaml의 `follower_node:` 절을 `/**`로 옮긴 임시 사본을 네 노드에 준다 |
| Isaac | `run_isaac_drive.sh --run-split` → `experiments/user_base/run_split.py` (`/ws/install`에 `line_follower_interfaces`가 빌드돼 있어야 한다) |
| 시험 | `src/line_follower/test/test_split_nodes.py` — 합성 60프레임에서 단일 지도와 미러 지도(grid/seen/conf/frozen)가 칸마다 같고 RulePlanner 경로가 같다 |

- 인지 단계(`stage.py`)는 그대로다. 분리 모드에서 `node.map`이 `FrameSink`라서 `freeze`/`update` 호출이 지도 대신 프레임으로 나간다.
- 패치 사각형은 `LineMap.update()`가 돌려주는 쓴 창이다. `map.last`의 `seen`은 frozen/add-only로 제자리에서 줄어들어 실제 쓴 범위보다 작을 수 있다.
- 0/255가 아닌 마스크(학습 플러그인의 bilinear BEV)는 packbits 대신 원래 uint8로 보낸다(길이로 구분). 그래서 지도 입력이 단일 노드와 같다.
- 분리 모드 미러에는 log-odds 증거 층이 없다. RulePlanner는 `confidence()`를 쓰지 않아 영향이 없지만, 쓰는 판단은 `map_mode: logodds`에서 값이 다르다.
- 프레임과 패치는 신뢰 QoS(깊이 100)다. 하나라도 빠지면 미러가 어긋나고, 1초마다 오는 전체 패치(`full=true`)가 다시 맞춘다.

## 6. 검증 (Isaac 공식 트랙, 2026-11-27)

분리 모드(`run_isaac_drive.sh --run-split`, V3 설정) 5/5 완주, 평균 랩 67.51 s, 14구간 모두 평균 횡오차 2 cm 이하(14.0/14). 같은 날 단일 노드 V3는 67.64 s, 13.3/14(실패 2회는 12.6 m 교차). 분리해도 주행 품질이 같다.

## 7. 단계별 처리 시간과 언어 결정 (2026-11-27)

같은 코드·같은 데이터(Isaac 시험 맵 프레임 40장, 인지 로그 300프레임)로 ROS 없이 단계만 잰 값이다(`experiments/jetson/bench_stages.py`, 결과 `bench_stages_pc.json`, `bench_stages_jetson.json`).

| 단계 | PC p50 / p95 | Jetson Orin Nano (MAXN_SUPER) p50 / p95 | 30 Hz 예산 33 ms 중 (Jetson) |
|---|---|---|---|
| 인지 (색 변환 + 규칙 BEV) | 1.6 / 1.7 ms | 3.6 / 3.6 ms | 11 % |
| 지도 갱신 (`LineMap.update`) | 0.5 / 0.7 ms | 2.4 / 2.5 ms | 7 % |
| 판단 더미 (`RulePlanner.replan`, `trace`) | 6.0 / 6.4 ms | 20.0 / 20.4 ms | 62 % |

Isaac 분리 모드 주행의 노드 로그(PC)도 같은 순서다: 인지 0.9 / 1.3 ms, 지도 1.2 / 1.5 ms, 판단 약 9 / 13 ms, 제어 0.16 / 0.25 ms.

- **인지·지도는 파이썬 그대로 둔다.** 무거운 계산은 이미 OpenCV·numpy(C 내부)가 해서, C++로 옮겨도 Jetson에서 몇 ms밖에 줄지 않는다.
- **C++로 옮길 후보는 판단의 경로 찾기**(`line_map.trace`와 판단 반복문)다. 파이썬 반복문이라 Jetson에서 프레임당 20 ms를 쓴다. 판단 팀이 새 판단 노드를 C++로 만들거나 `trace`만 C++ 확장으로 옮기는 것을 권한다.
- 러스트는 ROS 2 Jazzy 지원이 실험 단계라 쓰지 않는다.
