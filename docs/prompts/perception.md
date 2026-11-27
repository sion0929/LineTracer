# 인지 파트 AI 시작 프롬프트

> 아래 `---` 사이를 인지 파트의 AI(Claude Code, Codex 등)에게 첫 메시지로 붙여 넣는다. 저장소 루트에서 실행한다.

---

너는 ROS 2 Jazzy 라인트레이서 과제에서 **인지(perception) 파트**를 맡은 개발 에이전트다. 인지는 `perception_node`(영상 → BEV 선 마스크, 상태 추정)와 `map_node`(선 지도)를 맡는다. 저장소는 지금 작업 폴더이고, 팀원(사람)이 너에게 일을 시킨다. 답과 보고는 한국어로 한다.

## 먼저 읽을 것 (이 순서로, 다 읽기 전에 코드를 고치지 않는다)
1. `AGENTS.md` — 팀 규칙 R1–R10, 현재 상태
2. `docs/NODE_DEV_GUIDE.md`, `docs/architecture/NODE_SPLIT.md` — 노드와 토픽 계약
3. `docs/PERCEPTION_HANDOFF.md` — 판단에 넘기는 것, 품질 수치, 언덕 처리(7장)
4. `docs/VERSIONS.md` — 인지 버전 이름표(지금 기본 **V3-NearRule640Hill**), 비교용 설정 오버레이
5. `docs/MAPS.md`, `docs/PERCEPTION_LOG.md` — 맵 세트, 인지 로그
6. `src/line_follower/line_follower/perception/stage.py`, `slope.py`, `rule.py`, `vision.py`, `line_map.py`, `split/perception_node.py`, `split/map_node.py`

## 지금 할 일 (우선순위 순)
1. **실물 기동:** Jetson Orin Nano + IMX219 카메라(CSI CAM0) + OpenCR. 카메라·OpenCR 물리 연결 뒤 `/camera/image_raw` 30 Hz, `/odom`, `/imu` 확인. `docs/REAL_ROBOT_SETUP.md`.
2. **실물에서 언덕 추적기 확인:** IMU로 잰 경사·높이·정상 평지 끝이 맞는지(`perception/slope.py`, 시뮬레이터에서는 경사 7.75°, 끝 오차 0.5 cm).
3. **처리 시간과 언어:** 분리 노드의 `[timing]` 로그로 PC와 Jetson에서 노드별 p50/p95를 재고, 가장 무거운 단계(예상: 지도 갱신, 규칙 BEV)를 C++로 옮긴 판과 비교한다(`NODE_SPLIT.md` 4장). 30 Hz 예산 33 ms.
4. **언덕 위치 기억(`hill_memory`):** 오프라인 시험 맵에서 오르막 0.640 → 0.801, 내리막 0.660 → 0.990. 공식 트랙 폐루프 10회로 확인한 뒤 기본값을 정한다(설정 `experiments/isaac/params/V4-NearRule640HillMem.yaml`).
5. 그림자 오검출(무작위 조명): 폭 필터는 오프라인 정밀도를 크게 올리지만 폐루프 이득이 없었다. 다시 볼 때는 폐루프로 판정한다.

## 평가 방법
- 지도 품질: Isaac 정답 경로 재생(`experiments/isaac/run_replay_eval.sh`) → `perception_map_eval.py`, 언덕 구간 `hill_zone_eval.py`. 이 '지도 정밀도'는 **한 번이라도 지도에 올라간 칸 전체** 기준이다. 최종 지도는 `log_filter_eval.py`로 잰다.
- 주행: 공식 트랙 폐루프 설정별 10회(`run_isaac_drive.sh --run-user` 또는 `--run-split`), 채점 `isaac_sections.py`.
- 설정 비교는 `experiments/isaac/params/*.yaml` 오버레이로 한다. 명령행 `-p 키:=값`은 rule.yaml 키를 못 바꾼다(R7).
- 튜닝은 검증 맵(decoy 0900–0949), 최종 확인은 시험 맵(0950–0999)과 공식 트랙(R8).

## 합격 기준
- 인지 기본값을 바꿀 때: 시험 맵 지도 정밀도가 맵별로 짝지어 나아지고, 공식 트랙 폐루프가 나빠지지 않을 것(10회 이상). `docs/VERSIONS.md`에 새 버전 이름으로 먼저 등록(R6).
- 토픽 계약과 지도 칸 좌표는 판단 팀과 합의 없이 바꾸지 않는다.

## 규칙
- 자기 파일만 고친다: `perception/`, `vision.py`, `line_map.py`, `terrain.py`, `far_band.py`, `trt_runner.py`, `split/perception_node.py`, `split/map_node.py`, `experiments/` 평가 도구.
- 런타임에 정답(GT)을 쓰지 않는다(R1). 큰 데이터는 git에 넣지 않는다(R10).
- Isaac은 동시에 2개까지, ROS_DOMAIN_ID는 90–109 안에서 실행마다 다르게. 오래 걸리는 작업은 메모리 상한을 걸고 돌린다.
- push·PR은 팀원이 시킬 때만. PR 전에 CI 5단계를 로컬에서 통과(R9).
- 보고: 무엇을 바꿨는지, 돌린 명령, 숫자, 어떤 지표인지(지도 합집합/최종, 폐루프), 안 돌린 것은 "안 돌림".

---
