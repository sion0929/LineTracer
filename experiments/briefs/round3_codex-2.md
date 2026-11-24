# Codex-2 (딥러닝) — 3라운드

`git merge dev/two-track`을 먼저 해라.

**할 일**
1. `src/line_follower/line_follower/perception/dl.py`의 TODO(`load`, `segment`)를 채운다. U-Net v3(`experiments/dl/line_seg_v3.onnx`)를 onnxruntime **CPU**로 돌린다(채점 PC에 GPU가 없을 수 있다). 장애물은 v1이 낫다면 합쳐도 된다. 처리 시간 목표는 30 Hz 안이다(4스레드 약 18 ms로 측정됨). `config/dl.yaml`에 모델 경로를 넣는다. 모델 파일은 패키지 share에 설치되게 setup.py에 추가한다.
2. `mode:=dl`로 공식 3회와 `hills60`, `official_like50`을 bench로 재고, 리드의 `rule` 기준선(experiments/results/lead/A_baseline_*)과 비교한다.
3. Depth Anything V2 **Small**(Apache-2.0, ONNX)의 CPU 지연을 재고, 리드의 선 폭 기하 방법과 오프라인으로 비교한다(experiments/eval/hill_geometry_offline.py 방식, 언덕 점 위치 오차). 기하보다 확실히 낫지 않으면 채택하지 않는다.
4. 반복 실행은 DeepSeek 워커에게 맡긴다. 커밋은 track/b-dlseg에만 하고 push는 금지한다. 보고서는 experiments/reports/codex-2.md에 쓴다.
