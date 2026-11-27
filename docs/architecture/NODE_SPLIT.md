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
