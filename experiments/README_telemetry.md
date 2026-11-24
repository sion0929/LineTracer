# 주행 텔레메트리 (리드 추가)

`bench.py`는 모든 주행 폴더(`<output>/<NN_NN>/`)에 아래 파일을 자동으로 남긴다. x축은 **선을 따라 간 거리 s [m]** 이다.

| 파일 | 내용 |
|---|---|
| `telemetry.npz` / `telemetry.csv` | 약 50 ms 간격 시계열: t, x, y, yaw, s, progress(최대 도달), lat(부호 있는 횡오차, +는 진행방향 왼쪽), v, a, w, alpha, jerk, cmd_v, cmd_w, curv, wheel_on |
| `telemetry.json` | 요약: 최대 도달 거리/%, 평균·p95·최대 속도, 가속도, 각속도, 횡오차, 통로(±0.10/±0.12 m) 밖 시간, 바퀴 선 접촉 시간·횟수, 정지 시간, 완주/랩타임/종료 사유 |
| `telemetry.png` | 대시보드 (s 축 패널 6개 + 속도색 궤적 + 요약) |

CSV 결과에는 `wheel_line_events`, `wheel_line_sec`, `max_progress_m` 열이 추가됐다. 바퀴 기준: Waffle Pi 바퀴 중심 ±0.1435 m, 바퀴 반폭 0.009 m + 선 반폭 0.010 m.

```bash
python3 experiments/telemetry.py plot RUN_OR_BENCH_DIR           # 그림 다시 그리기
python3 experiments/telemetry.py compare out.png DIR [DIR ...]   # 여러 주행 겹쳐 비교 + 최대 진행률 막대
```
지상 진실(x, y, yaw)은 채점용 개발 도구에서만 쓰며 로봇 런타임에는 쓰지 않는다.

## 한 컨테이너에서 bench 여러 개 동시 실행
각 bench는 실행마다 ROS 도메인을 30번부터 차례로 나눠 준다. bench를 동시에 두 개 이상 띄우면 `BENCH_DOMAIN_BASE`로 시작 번호를 겹치지 않게 지정한다(예: 첫 번째 30, 두 번째 50, 세 번째 70). `GZ_PARTITION`도 서로 다르게 준다.
