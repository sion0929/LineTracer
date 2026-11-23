# Codex-2 지시서

너는 Codex-2 (딥러닝 인식). LAB_NAME=lf_b, ROS_DOMAIN_ID=12, GZ_PARTITION=lf_b.
워크트리: /home/rvlab/orca/workspaces/26_robotprogramming_project/b-dlseg, 브랜치: track/b-dlseg (여기에만 커밋).
(ORCA_PLAN 원문의 codexB/82 같은 이름·도메인 값은 무시하고 위 값을 써라. 모든 컨테이너·sim 명령에 LAB_NAME=lf_b, ROS_DOMAIN_ID=12, GZ_PARTITION=lf_b를 지정.)

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
RTX 3090 사용 가능(호스트 miniforge에 PyTorch 환경 만들 것).
딥러닝 인식 트랙. 산출물은 experiments/dl/ 아래만.
(1) 데이터 수집기: 맵(처음엔 공식 트랙, 이후 Codex-3의 experiments/worlds/ 랜덤 맵)에서 로봇을 선 근처 다양한 자세(횡오프셋 ±0.1 m, 각도 ±40°, 언덕 위, 장애물 앞)로 텔레포트하며 영상 수집. 라벨(배경/라인/장애물)은 알려진 기하(중심선 폴리라인, 장애물 박스)를 카메라 모델로 투영해 자동 생성. 수천~수만 장.
(2) 분할 모델: Fast-SCNN 또는 소형 UNet을 학습. 조명·색 증강 포함. 평가: 공식 트랙(학습 제외)과 처음 보는 랜덤 맵에서 클래스별 IoU, OpenCV 임계값(gray<80) 대비 비교. 추론 30 Hz 이상(CPU와 GPU 둘 다 측정), ONNX 내보내기.
(3) 연동: experiments/dl/infer.py에 segment(bgr_img) -> (line_mask, obstacle_mask) 640x480 bool. follower.py가 --perception dl 옵션으로 쓸 수 있게 사용 예 제공(follower.py 수정은 하지 말고 Codex-1에게 넘길 인터페이스만).
(4) 선택: 마스크→조향 모방학습(선생님=follower.py) 실험은 (1)~(3) 완료 후에만.
완료 기준: experiments/dl/REPORT.md에 데이터 규모, IoU 표, 속도, 실패 예시 이미지.

보고: 단계마다 리드(LEAD)에게 변경 요약·측정 수치(결과 파일 경로 포함)·남은 문제를 한국어로 보고. 수치는 리드가 직접 재현 확인한 뒤 dev/two-track에 병합한다.
