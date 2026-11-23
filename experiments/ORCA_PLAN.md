# LineTracer 2주 투트랙 계획 (Orca 배포용)

## 0. 시작 전 1회 (리드가 실행)
1. 지금 작업물을 브랜치로 고정 (워크트리 에이전트가 `experiments/`, `docker/lab_substitute/`를 볼 수 있게):
   `git switch -c dev/two-track && git add experiments docker/lab_substitute && git commit -m "experiments: map-based follower, eval harness, lab image substitute"`
2. `tools/lab_env.sh`의 `NAME=line_follower_lab` → `NAME=${LAB_NAME:-line_follower_lab}` 로 바꿔 커밋 (워크트리마다 컨테이너 분리).
3. 각 에이전트는 `dev/two-track`에서 자기 워크트리/브랜치를 만들어 작업.

## 편성표 (Orca에 생성 완료)
| 이름 | 에이전트 | Orca 워크트리 / 탭 | 브랜치 | LAB_NAME | ROS_DOMAIN_ID | GZ_PARTITION |
|---|---|---|---|---|---|---|
| LEAD | Claude Code (충북대 게이트웨이) | `LEAD · 오케스트레이터` (메인 체크아웃) / 탭 `LEAD · Claude(CBNU)` | `dev/two-track` | lf_lead | 10 | lf_lead |
| Codex-1 | Codex | `A · 추종기 (Codex-1)` = `a-follower` | `track/a-follower` | lf_a | 11 | lf_a |
| Codex-2 | Codex | `B · 딥러닝 인식 (Codex-2)` = `b-dlseg` | `track/b-dlseg` | lf_b | 12 | lf_b |
| Codex-3 | Codex | `C · 맵·채점·스윕 (Codex-3)` = `c-bench` | `track/c-bench` | lf_c | 13 | lf_c |
| DS-A / DS-B / DS-C | DeepSeek V4.1 Flash eco (`opencode run`) | 상주 탭 없음: 각 Codex가 자기 워크트리에서 일회성으로 호출 | 커밋 안 함 | 호출한 Codex 것 사용 | 호출한 Codex 것 | 호출한 Codex 것 |
| DS-NIGHT | DeepSeek (프롬프트 D) | `c-bench`에 탭 `DS-NIGHT · 야간스윕` (sweep.py 완성 후 생성) | 커밋 안 함 | lf_sweep | 20 | lf_sweep |

워커 결과 파일 규칙: `experiments/results/<DS-이름>_<YYYYMMDD-HHMM>_<내용>.{json,log,md}`.

## 공통 컨텍스트 (모든 에이전트 프롬프트 맨 앞에 붙이기)
```
[공통 컨텍스트] 한국어로 보고할 것.
프로젝트: ROS 2 Jazzy + Gazebo Harmonic 라인트레이서(TurtleBot3 Waffle Pi). 원본은 친구 코드(github sion0929/LineTracer), 규칙 기반이며 공식 트랙에서 28%(4.45 m/15.7 m, 177 s)만 진행. 목표: 라인에 최대한 붙어서 최대한 빠르게. 평가 맵은 제공 트랙과 다르고 더 어려우며 상자형 장애물(라인을 벗어나 회피 → 라인 복귀)이 나올 예정(공식 규칙 미공개, 상자 가정). 기간 2주.
과제 규칙: 로봇 런타임은 /camera/image_raw, /imu, /odom, /cmd_vel만 사용. /world/ 토픽·set_pose 등 시뮬 정답은 개발 도구(채점·데이터 수집)에서만. 제공 src/line_follower_bringup/worlds, models 원본 수정 금지(SHA256 검사). 최종 실행은 `ros2 launch line_follower_bringup competition.launch.py` 한 줄, 기동 20초 후 출발, /cmd_vel은 safety_guard만 발행.
실행 환경: 호스트엔 ROS 없음. Docker 랩 컨테이너 사용: `LAB_NAME=lf_<내이름> tools/lab_env.sh build` (이미지 rp/ros2:jazzy-lab = docker/lab_substitute/Dockerfile 대체 빌드, gz python 포함). docker 권한 오류 시 `sg docker -c "..."`로 감싸기. 컨테이너 안 실행: `docker exec lf_<내이름> bash -c 'source /opt/ros/jazzy/setup.bash; source /ws/install_lab/setup.bash; cd /ws && <명령>'`.
동시 실행 규칙: 내 고유 ROS_DOMAIN_ID와 GZ_PARTITION만 사용, `pkill gz` 같은 전역 종료 금지(자기 프로세스 그룹만 정리).
카메라: 차축 앞 0.076 m, 높이 0.103 m, 피치 0, hfov 1.085595, 640x480 30 Hz. 선: 베이지 매트 위 검은 선 폭 약 2 cm. 로봇 앞 0.3 m는 카메라 사각지대.
기존 실험 코드(experiments/): follower.py(지도 기반 추종기), run_eval.py(헤드리스 1회 주행 채점: 진행률·랩타임·횡오차, 0.08 m 0.6 s 이탈 시 실패), run.sh(실행 래퍼), calib.py(텔레포트·투영 검증), dbg.py(지도 디버그 그림), gt.npy/segments.npy(공식 트랙 중심선).
git: 내 브랜치에만 커밋, main/dev/two-track 직접 수정 금지. 끝나면 변경 요약·측정 수치·남은 문제를 보고.
[워커 위임 규칙] 시뮬레이션 반복 실행·스윕·장시간 대기는 직접 붙잡고 있지 말고 DeepSeek 워커에게 맡겨라: `opencode run -m opengateway/deepseek/deepseek-v4.1-flash-eco "<무엇을 실행·몇 번·결과를 어느 파일에 어떤 형식으로·코드 수정 금지·에러 3회면 중단하고 로그 요약>"`. 워커 결과는 반드시 결과 파일 수치로 직접 확인하고 보고해라.
```

## 3단 구조: 오케스트레이터(Claude, 충북대) → 작업자(Codex) → 워커(DeepSeek)
| 층 | 누가 | 하는 일 | 하지 않는 일 |
|---|---|---|---|
| 오케스트레이터 | Claude Code (충북대 AI Gateway) | 계획·지시서 작성, Codex 작업 배정, 결과 수치 재현 확인, 병합·게이트 판정 | 직접 대량 코딩, 반복 실행 |
| 작업자 | Codex ×3 (Codex-1/2/3) | 설계·구현·디버깅, 워커에게 반복 실행 위임 | 남의 소유 파일 수정, 병합 |
| 워커 | DeepSeek V4.1 Flash eco (OpenCode, Sionic 서빙) | 정해진 스크립트 반복 실행·재시작·로그 요약 | 코드(.py) 수정, 판단 |

오케스트레이터 실행(Orca 터미널, 충북대 게이트웨이로 Claude Code 기동):
`ANTHROPIC_BASE_URL=https://factchat-cloud.mindlogic.ai/v1/gateway/claude ANTHROPIC_API_KEY="$CBNU_API_KEY" claude`
워커 호출(작업자 Codex가 자기 터미널에서 실행):
`opencode run -m opengateway/deepseek/deepseek-v4.1-flash-eco "<지시>"`

| 작업자 | 트랙 | 소유 파일 |
|---|---|---|
| Codex-1 | A. 새 추종기(규칙/지도 기반) | `experiments/follower.py`, `experiments/obstacle.py`, 이후 `src/line_follower_fastfollow/` |
| Codex-2 | B. 딥러닝 인식 | `experiments/dl/` |
| Codex-3 | 공용 인프라: 랜덤 맵·다중 맵 채점·파라미터 탐색 | `experiments/worlds/`, `experiments/bench.py`, `experiments/sweep.py` |

## 일정·게이트
| 기간 | A (Codex-1) | B (Codex-2) | C (Codex-3) |
|---|---|---|---|
| 1~3일 | 공식 트랙 5/5 완주 | 데이터 수집기(공식 트랙 먼저) | 랜덤 맵 생성기 + bench.py |
| 4~7일 | 속도 향상 + 장애물 회피 v1 | 랜덤 맵 데이터 수천 장, 분할 모델 v1 | sweep.py(Optuna) |
| 8~11일 | 랜덤 맵 일반화, DL 마스크 입력 옵션 | 모델 v2, ONNX, segment() 연동 | 야간 스윕 운영(DeepSeek에 인계) |
| 12~14일 | ROS 패키지화·competition.launch 통합 | 최종 모델 고정 | 최종 벤치(규칙 vs 딥러닝 vs 원본) |

게이트(리드 확인): G1(3일) 공식 트랙 완주 / G2(7일) 랜덤 맵 20개 평균 완주율 측정 / G3(11일) 규칙 vs 딥러닝 비교표 / G4(14일) Docker 한 줄 실행 검증.

---

## 프롬프트 A — Codex-1: 새 추종기
```
(공통 컨텍스트 붙이기) 내 이름: codexA, ROS_DOMAIN_ID=81, GZ_PARTITION=codexA_*.
experiments/follower.py(지도 기반 추종기)를 완성해. 구조: 어두운 픽셀(gray<80)을 지면 투영 → 오도메트리 거리+IMU yaw 추측항법으로 odom 격자 지도(5 mm, 누적 점수 hit+3/miss-1)에 기록 → 로봇 위치에서 선을 추적(갈림길은 가장 곧은 방향, T자는 끝) → Pure Pursuit + 곡률 기반 감속.
현재 상태(실측): 언덕 포함 첫 직선 2.6 m를 ~14 s, 횡오차 ~0으로 통과. 첫 직각 코너(트랙 2.6~2.7 m)에서 실패.
밝혀진 사실: 정지 투영 오차 5 mm로 정확(calib.py). 오도메트리 yaw는 제자리 회전 시 미끄러져 틀림 → IMU yaw 사용(적용됨). 언덕에서 오도메트리 거리 약 +4 cm 누적(무해). 코너 급회전 시 바퀴 오도메트리는 정지인데 실제 로봇은 ~0.2 m 미끄러지는 물리 이상 관측(t10, 0.26 m/s 슬립) → 방금 가감속 제한(--waccel 4, --decel-cmd 0.8, --wmax 1.5)을 넣었고 미검증. 전방 주시 거리가 크면 반지름 0.1 m 코너를 잘라먹음(--ld-min 0.04 --ld-gain 0.2 --alat 0.15로 시험 중).
할 일: (1) experiments/run.sh로 반복 시험, dbg.py와 덤프(trail, cmds)로 원인 확인하며 공식 트랙 5회 연속 완주. (2) 완주 후 vmax/alat/lookahead를 올려 랩타임 단축, 횡오차 p95 ≤ 2 cm 유지. (3) Codex-3의 랜덤 맵(experiments/worlds/)이 나오면 일반화 시험. (4) 장애물 회피 v1(experiments/obstacle.py): 지면 ROI에서 베이지도 검정도 아닌 영역/지면 위로 솟은 물체 검출 → 좌우 여유 공간 비교 → 옆으로 비켜 지나간 뒤 기억 지도/재검출로 라인 복귀. (5) 마지막 주에 ROS 패키지(src/line_follower_fastfollow)로 옮기고 safety_guard 경유 /cmd_vel, mission_manager 20초 게이트와 연결.
완료 기준: 공식 트랙 5/5 완주, 랩타임·횡오차 p95 표, 랜덤 맵 결과, 장애물 회피 성공률. 매 단계 수치로 보고.
```

## 프롬프트 B — Codex-2: 딥러닝 인식
```
(공통 컨텍스트 붙이기) 내 이름: codexB, ROS_DOMAIN_ID=82, GZ_PARTITION=codexB_*. RTX 3090 사용 가능(호스트 miniforge에 PyTorch 환경 만들 것).
딥러닝 인식 트랙. 산출물은 experiments/dl/ 아래만.
(1) 데이터 수집기: 맵(처음엔 공식 트랙, 이후 Codex-3의 experiments/worlds/ 랜덤 맵)에서 로봇을 선 근처 다양한 자세(횡오프셋 ±0.1 m, 각도 ±40°, 언덕 위, 장애물 앞)로 텔레포트하며 영상 수집. 라벨(배경/라인/장애물)은 알려진 기하(중심선 폴리라인, 장애물 박스)를 카메라 모델로 투영해 자동 생성. 수천~수만 장.
(2) 분할 모델: Fast-SCNN 또는 소형 UNet을 학습. 조명·색 증강 포함. 평가: 공식 트랙(학습 제외)과 처음 보는 랜덤 맵에서 클래스별 IoU, OpenCV 임계값(gray<80) 대비 비교. 추론 30 Hz 이상(CPU와 GPU 둘 다 측정), ONNX 내보내기.
(3) 연동: experiments/dl/infer.py에 segment(bgr_img) -> (line_mask, obstacle_mask) 640x480 bool. follower.py가 --perception dl 옵션으로 쓸 수 있게 사용 예 제공(follower.py 수정은 하지 말고 Codex-1에게 넘길 인터페이스만).
(4) 선택: 마스크→조향 모방학습(선생님=follower.py) 실험은 (1)~(3) 완료 후에만.
완료 기준: experiments/dl/REPORT.md에 데이터 규모, IoU 표, 속도, 실패 예시 이미지.
```

## 프롬프트 C — Codex-3: 랜덤 맵·다중 맵 채점·파라미터 탐색
```
(공통 컨텍스트 붙이기) 내 이름: codexC, ROS_DOMAIN_ID=83, GZ_PARTITION=codexC_*.
공용 실험 인프라를 만들어. A·B 트랙이 이걸 쓰니 가장 먼저 (1)을 끝내고 README로 사용법을 알릴 것.
(1) experiments/worlds/gen_world.py --seed N --difficulty {easy,normal,hard} --obstacles K: 공식 트랙과 같은 스타일(같은 매트 색, 같은 선 폭·높이, 같은 메시 방식)로 곡선(다양한 반지름)·직각 코너·교차로·루프·경사로를 섞은 연결된 트랙 + 랜덤 상자 장애물(크기·색·위치, 선 위 포함)을 가진 SDF 월드 생성. 출발 T자·도착 T자 포함. 중심선(순서 있는 폴리라인)·장애물 목록·출발 자세를 JSON으로 저장. sim에서 로드되고 로봇이 출발점에 스폰되는지 실제 확인. 공식 트랙과 비슷한 난이도 분포.
(2) experiments/bench.py: 맵 목록 × 추종기 인자로 헤드리스 주행을 병렬(동시 N개, 각자 도메인/파티션) 실행하고, run_eval.py와 같은 기준(진행률·완주·랩타임·횡오차 p95·장애물 충돌/회피)으로 채점해 CSV/표 출력. 장애물 충돌 판정 포함. run_eval.py는 수정 말고 필요한 기능은 bench.py에 구현.
(3) experiments/sweep.py: Optuna로 follower.py 파라미터(vmax, alat, ld-min, ld-gain, accel, waccel 등)를 랜덤 맵 묶음 평균 점수(완주율 우선, 그다음 랩타임, 횡오차 페널티)로 최적화. 중단 후 재개 가능(SQLite 스토리지), 결과는 experiments/sweeps/<날짜>/에 저장, 진행 요약 summary.md 자동 갱신.
(4) 물리 이상 감시: 실제 자세 대비 바퀴 오도메트리 미끄럼(슬립) 이벤트를 bench 결과에 기록.
완료 기준: 생성 맵 20개 미리보기 그림, bench로 follower.py 기본 인자 결과표, sweep 10회 시험 실행 로그.
```

## 프롬프트 D — DeepSeek 워커: 야간 스윕 운영 (Codex-3이 sweep.py 완성 후 자기 터미널에서 호출)
```
opencode run -m opengateway/deepseek/deepseek-v4.1-flash-eco "experiments/sweeps/README.md의 절차대로 experiments/sweep.py를 실행해. 프로세스가 죽거나 30분간 trial이 진행되지 않으면 같은 스토리지로 재시작해. 1시간마다 experiments/sweeps/<날짜>/summary.md에 상위 5개 파라미터와 점수를 표로 갱신해. 코드(.py)는 절대 수정하지 말고, 에러가 3번 반복되면 멈추고 에러 로그만 요약해."
```

## 프롬프트 E — 오케스트레이터(Claude, 충북대 게이트웨이)
```
(공통 컨텍스트 붙이기) 너는 오케스트레이터다. 직접 대량 코딩이나 시뮬 반복 실행은 하지 말고, 작업은 Codex 작업자 3명에게, 반복 실행은 그들이 DeepSeek 워커에게 맡기게 해라. 계획서: experiments/ORCA_PLAN.md.
1) 0단계(브랜치 dev/two-track 커밋, lab_env.sh LAB_NAME 지원)가 됐는지 확인하고 안 됐으면 수행.
2) Orca에서 Codex 작업자 3개를 dev/two-track 기반 별도 워크트리로 띄우고 프롬프트 A/B/C(공통 컨텍스트 포함)를 전달. Codex-3의 랜덤 맵 생성기가 1순위라고 A/B에 알려라.
3) 작업자 보고가 오면: 보고 수치를 해당 브랜치에서 run_eval.py 또는 bench.py로 1회 직접 재현 확인 → 통과하면 dev/two-track에 병합, 아니면 원인과 다음 지시를 구체적으로 돌려보내라. 두 작업자가 같은 파일을 건드리지 않게 소유 파일 표를 지켜라.
4) 게이트 G1(3일 공식 트랙 완주) G2(7일 랜덤 맵 20개 평균) G3(11일 규칙 vs 딥러닝 비교표) G4(14일 Docker 한 줄 실행)를 판정하고 매일 experiments/STATUS.md에 진행·수치·막힌 점을 갱신.
5) 12일차: 원본 코드 / 규칙 추종기 / 규칙+딥러닝 인식을 같은 랜덤 맵 20개+공식 트랙에서 비교해 최종 구성 결정, competition.launch.py 통합과 Docker 실행 검증을 Codex-1에게 지시.
```
