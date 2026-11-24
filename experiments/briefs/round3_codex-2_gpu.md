# Codex-2 추가 지시: GPU(RTX 3090)를 쉬지 않게 쓴다

지금 3090 사용률이 약 25%로 거의 놀고 있다. 3라운드 일과 병행해서 아래 GPU 작업을 **백그라운드 대기열로 이어서** 돌려라. 학습이 끝나면 다음 작업이 바로 시작되게 한다(nohup이나 간단한 큐 스크립트). 학습 감시와 로그 요약은 DeepSeek 워커에게 맡긴다.

1. **dataset-v2 수집 (Gazebo, CPU):** 새 맵 세트 general200, official_like50, hills60, obstacles100에서 수집한다(언덕 여러 개, 코너 원, 연결 교차로, 교차 막대, 곁가지, 끊긴 길, 반전, 상자). 시드 1000 이상만 쓰고, 0–39와 평가용 hills60 일부는 시험용으로 남긴다. bench 도메인 규칙(lf_b 블록 50–69)을 지킨다. 수집 중에도 GPU는 2번 학습으로 쓴다.
2. **v4 다중 출력 모델 학습 (GPU):** 출력은 라인과 장애물 분할 + **픽셀별 깊이(또는 지면 높이) 회귀 헤드**다. 정답 깊이는 experiments/dl/geometry.py의 `render_mask(..., return_depth=True)`로 만든다. 이것이 언덕 문제(평면 IPM 오차)를 학습으로 푸는 우리만의 "3D 라인" 버전이다. 증강은 Orin 실카메라 대비로 블러, 노이즈, 색, 감마, 그림자를 넣는다. 크기는 CPU 30 Hz와 Orin TensorRT를 생각해 v3 정도로 유지한다.
3. **v4 평가:** 오프라인으로 라인 IoU(experiments/eval/perception_offline.py 방식)와 언덕 점 3D 위치 오차를 잰다. 기준은 선 폭 기하 1.17 cm, IPM 21.8 cm다(experiments/eval/hill_geometry_offline.py, docs/OPEN_SOURCE_SURVEY.md 4장).
4. **남는 GPU 시간:** 잔차 강화학습 장기 학습(experiments/rl, 200만 스텝 이상)과 Depth Anything V2 Small 미세조정 비교.
5. 매 단계 결과를 experiments/reports/codex-2.md에 기록한다. GPU 사용률이 60% 아래로 오래 떨어지면 다음 작업을 넣는다.

## 운영 규칙 추가 (사용자 지시)
- **GPU 사용률 약 90%와 메모리 약 21 GB(24 GB의 90%)까지만 쓴다.** 나머지는 화면과 Gazebo 렌더링 여유로 남긴다.
- **작업 하나만 크게 돌리지 말고 여러 작업을 병렬로 돌린다.** 예: v4 변형 2–3개(크기, 손실 가중치, 증강 강도) 동시 학습 + 잔차 강화학습 + Depth Anything V2 Small 평가. 프로세스마다 배치 크기를 조절해 합계가 위 한도 안에 들게 한다.
- 간단한 감시 스크립트를 둔다. `nvidia-smi`로 사용률과 메모리를 30초마다 보고, 한도 아래에 여유가 생기면 대기열의 다음 작업을 띄운다. 한도를 넘으면 새 작업을 띄우지 않는다. 감시 로그 요약은 DeepSeek 워커에게 맡긴다.
