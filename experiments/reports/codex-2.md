## 2026-11-23 22:13 KST · 수집기 검증

- `lf_b` Lab 빌드 8개 패키지 완료. 호스트 Miniforge 경로의 `lf_b_dl` 환경에서 RTX 3090과 PyTorch 2.11 CUDA 인식 확인.
- 공식 트랙과 Codex-3 랜덤 맵에서 각각 16장 수집. 공식 라인 70,209픽셀, 랜덤 라인 78,061픽셀 및 장애물 121,605픽셀. `experiments/dl/runs/{official-smoke2,random-smoke}/summary.json` 참조. 샘플 영상과 오버레이를 육안으로 비교했다.
- 대량 수집은 학습 2,100장, 검증 180장, 랜덤 시험 360장, 공식 시험 180장으로 배정했다. 이 수치는 계획치다.
