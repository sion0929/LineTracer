# Codex-3 (인프라) — 3라운드: 제출용 Docker (G4)

`git merge dev/two-track`을 먼저 해라.

**할 일**
1. 사용자 패키지 `src/line_follower`를 제출 형태로 만든다. `ros2 launch line_follower line_follower.launch.py` 한 줄로 Gazebo, 브리지, 노드가 뜨고 20초 뒤 출발해야 한다. 원본 worlds/models는 SHA256이 같아야 한다.
2. 제출용 Dockerfile(`src/line_follower/Dockerfile.submission` 참고)을 만든다. 베이스는 rp/ros2:jazzy-lab이고, onnxruntime **CPU** 버전과 모델 파일을 이미지에 넣는다. GPU가 없는 PC에서 `mode:=rule`과 `mode:=dl` 둘 다 실행되는지 확인한다(`--gpus` 없이 실행).
3. 채점 PC가 느릴 수 있으니 Gazebo 실시간 배율(real time factor)과 노드 처리율을 컨테이너 안에서 측정해 보고한다.
4. bench 도메인 규칙을 지킨다(lf_c 블록 70–89). 커밋은 track/c-bench에만 하고 push는 금지한다. 보고서는 experiments/reports/codex-3.md에 쓴다.
