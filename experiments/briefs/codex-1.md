# Codex-1 지시서

너는 Codex-1 (추종기). LAB_NAME=lf_a, ROS_DOMAIN_ID=11, GZ_PARTITION=lf_a.
워크트리: /home/rvlab/orca/workspaces/26_robotprogramming_project/a-follower, 브랜치: track/a-follower (여기에만 커밋).
(ORCA_PLAN 원문의 codexA/81 같은 이름·도메인 값은 무시하고 위 값을 써라. 모든 컨테이너·sim 명령에 LAB_NAME=lf_a, ROS_DOMAIN_ID=11, GZ_PARTITION=lf_a를 지정.)

※ 우선순위: Codex-3의 랜덤 맵 생성기(experiments/worlds/gen_world.py)가 전체 1순위다. 나오기 전에는 공식 트랙으로 진행하고, 나오면 c-bench 브랜치의 experiments/worlds/README.md를 보고 바로 사용해라. 그쪽 파일은 수정하지 말 것.

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
experiments/follower.py(지도 기반 추종기)를 완성해. 구조: 어두운 픽셀(gray<80)을 지면 투영 → 오도메트리 거리+IMU yaw 추측항법으로 odom 격자 지도(5 mm, 누적 점수 hit+3/miss-1)에 기록 → 로봇 위치에서 선을 추적(갈림길은 가장 곧은 방향, T자는 끝) → Pure Pursuit + 곡률 기반 감속.
현재 상태(실측): 언덕 포함 첫 직선 2.6 m를 ~14 s, 횡오차 ~0으로 통과. 첫 직각 코너(트랙 2.6~2.7 m)에서 실패.
밝혀진 사실: 정지 투영 오차 5 mm로 정확(calib.py). 오도메트리 yaw는 제자리 회전 시 미끄러져 틀림 → IMU yaw 사용(적용됨). 언덕에서 오도메트리 거리 약 +4 cm 누적(무해). 코너 급회전 시 바퀴 오도메트리는 정지인데 실제 로봇은 ~0.2 m 미끄러지는 물리 이상 관측(t10, 0.26 m/s 슬립) → 방금 가감속 제한(--waccel 4, --decel-cmd 0.8, --wmax 1.5)을 넣었고 미검증. 전방 주시 거리가 크면 반지름 0.1 m 코너를 잘라먹음(--ld-min 0.04 --ld-gain 0.2 --alat 0.15로 시험 중).
할 일: (1) experiments/run.sh로 반복 시험, dbg.py와 덤프(trail, cmds)로 원인 확인하며 공식 트랙 5회 연속 완주. (2) 완주 후 vmax/alat/lookahead를 올려 랩타임 단축, 횡오차 p95 ≤ 2 cm 유지. (3) Codex-3의 랜덤 맵(experiments/worlds/)이 나오면 일반화 시험. (4) 장애물 회피 v1(experiments/obstacle.py): 지면 ROI에서 베이지도 검정도 아닌 영역/지면 위로 솟은 물체 검출 → 좌우 여유 공간 비교 → 옆으로 비켜 지나간 뒤 기억 지도/재검출로 라인 복귀. (5) 마지막 주에 ROS 패키지(src/line_follower_fastfollow)로 옮기고 safety_guard 경유 /cmd_vel, mission_manager 20초 게이트와 연결.
완료 기준: 공식 트랙 5/5 완주, 랩타임·횡오차 p95 표, 랜덤 맵 결과, 장애물 회피 성공률. 매 단계 수치로 보고.

보고: 단계마다 리드(LEAD)에게 변경 요약·측정 수치(결과 파일 경로 포함)·남은 문제를 한국어로 보고. 수치는 리드가 직접 재현 확인한 뒤 dev/two-track에 병합한다.
