# Codex-3 지시서

너는 Codex-3 (맵·채점·스윕). LAB_NAME=lf_c, ROS_DOMAIN_ID=13, GZ_PARTITION=lf_c.
워크트리: /home/rvlab/orca/workspaces/26_robotprogramming_project/c-bench, 브랜치: track/c-bench (여기에만 커밋).
(ORCA_PLAN 원문의 codexC/83 같은 이름·도메인 값은 무시하고 위 값을 써라. 모든 컨테이너·sim 명령에 LAB_NAME=lf_c, ROS_DOMAIN_ID=13, GZ_PARTITION=lf_c를 지정.)

※ 네 (1) 랜덤 맵 생성기가 A·B 모두를 막고 있는 전체 1순위다. (1)을 먼저 끝내 커밋하고 experiments/worlds/README.md에 사용법을 쓴 뒤 리드에게 알려라.

[공통 컨텍스트] 한국어로 보고할 것.
프로젝트: ROS 2 Jazzy + Gazebo Harmonic 라인트레이서(TurtleBot3 Waffle Pi). 원본은 친구 코드(github sion0929/LineTracer), 규칙 기반이며 공식 트랙에서 28%(4.45 m/15.7 m, 177 s)만 진행. 목표: 라인에 최대한 붙어서 최대한 빠르게. 평가 맵은 제공 트랙과 다르고 더 어려우며 상자형 장애물(라인을 벗어나 회피 → 라인 복귀)이 나올 예정(공식 규칙 미공개, 상자 가정). 기간 2주.
과제 규칙: 로봇 런타임은 /camera/image_raw, /imu, /odom, /cmd_vel만 사용. /world/ 토픽·set_pose 등 시뮬 정답은 개발 도구(채점·데이터 수집)에서만. 제공 src/line_follower_bringup/worlds, models 원본 수정 금지(SHA256 검사). 최종 실행은 `ros2 launch line_follower_bringup competition.launch.py` 한 줄, 기동 20초 후 출발, /cmd_vel은 safety_guard만 발행.
실행 환경: 호스트엔 ROS 없음. Docker 랩 컨테이너 사용: `LAB_NAME=lf_<내이름> tools/lab_env.sh build` (이미지 rp/ros2:jazzy-lab = docker/lab_substitute/Dockerfile 대체 빌드, gz python 포함). docker 권한 오류 시 `sg docker -c "..."`로 감싸기. 컨테이너 안 실행: `docker exec lf_<내이름> bash -c 'source /opt/ros/jazzy/setup.bash; source /ws/install_lab/setup.bash; cd /ws && <명령>'`.
동시 실행 규칙: 내 고유 ROS_DOMAIN_ID와 GZ_PARTITION만 사용, `pkill gz` 같은 전역 종료 금지(자기 프로세스 그룹만 정리).
카메라: 차축 앞 0.076 m, 높이 0.103 m, 피치 0, hfov 1.085595, 640x480 30 Hz. 선: 베이지 매트 위 검은 선 폭 약 2 cm. 로봇 앞 0.3 m는 카메라 사각지대.
기존 실험 코드(experiments/): follower.py(지도 기반 추종기), run_eval.py(헤드리스 1회 주행 채점: 진행률·랩타임·횡오차, 0.08 m 0.6 s 이탈 시 실패), run.sh(실행 래퍼), calib.py(텔레포트·투영 검증), dbg.py(지도 디버그 그림), gt.npy/segments.npy(공식 트랙 중심선).
git: 내 브랜치에만 커밋, main/dev/two-track 직접 수정 금지. push·PR 생성 절대 금지(로컬 커밋만, pre-push 훅이 막음, --no-verify 우회 금지). 원격 업로드는 사용자가 직접 지시할 때만 리드가 한다. 끝나면 변경 요약·측정 수치·남은 문제를 보고.
[워커 위임 규칙] 시뮬레이션 반복 실행·스윕·장시간 대기는 직접 붙잡고 있지 말고 DeepSeek 워커에게 맡겨라: `opencode run -m opengateway/deepseek/deepseek-v4.1-flash-eco "<무엇을 실행·몇 번·결과를 어느 파일에 어떤 형식으로·코드 수정 금지·에러 3회면 중단하고 로그 요약>"`. 워커 결과는 반드시 결과 파일 수치로 직접 확인하고 보고해라.

## 작업
공용 실험 인프라를 만들어. A·B 트랙이 이걸 쓰니 가장 먼저 (1)을 끝내고 README로 사용법을 알릴 것.
(1) experiments/worlds/gen_world.py --seed N --difficulty {easy,normal,hard} --obstacles K: 공식 트랙과 같은 스타일(같은 매트 색, 같은 선 폭·높이, 같은 메시 방식)로 곡선(다양한 반지름)·직각 코너·교차로·루프·경사로를 섞은 연결된 트랙 + 랜덤 상자 장애물(크기·색·위치, 선 위 포함)을 가진 SDF 월드 생성. 출발 T자·도착 T자 포함. 중심선(순서 있는 폴리라인)·장애물 목록·출발 자세를 JSON으로 저장. sim에서 로드되고 로봇이 출발점에 스폰되는지 실제 확인. 공식 트랙과 비슷한 난이도 분포.
(2) experiments/bench.py: 맵 목록 × 추종기 인자로 헤드리스 주행을 병렬(동시 N개, 각자 도메인/파티션) 실행하고, run_eval.py와 같은 기준(진행률·완주·랩타임·횡오차 p95·장애물 충돌/회피)으로 채점해 CSV/표 출력. 장애물 충돌 판정 포함. run_eval.py는 수정 말고 필요한 기능은 bench.py에 구현.
(3) experiments/sweep.py: Optuna로 follower.py 파라미터(vmax, alat, ld-min, ld-gain, accel, waccel 등)를 랜덤 맵 묶음 평균 점수(완주율 우선, 그다음 랩타임, 횡오차 페널티)로 최적화. 중단 후 재개 가능(SQLite 스토리지), 결과는 experiments/sweeps/<날짜>/에 저장, 진행 요약 summary.md 자동 갱신.
(4) 물리 이상 감시: 실제 자세 대비 바퀴 오도메트리 미끄럼(슬립) 이벤트를 bench 결과에 기록.
완료 기준: 생성 맵 20개 미리보기 그림, bench로 follower.py 기본 인자 결과표, sweep 10회 시험 실행 로그.

보고: 단계마다 리드(LEAD)에게 변경 요약·측정 수치(결과 파일 경로 포함)·남은 문제를 한국어로 보고. 수치는 리드가 직접 재현 확인한 뒤 dev/two-track에 병합한다.
