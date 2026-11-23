## 2026-11-23 22:13 KST · 수집기 검증

- `lf_b` Lab 빌드 8개 패키지 완료. 호스트 Miniforge 경로의 `lf_b_dl` 환경에서 RTX 3090과 PyTorch 2.11 CUDA 인식 확인.
- 공식 트랙과 Codex-3 랜덤 맵에서 각각 16장 수집. 공식 라인 70,209픽셀, 랜덤 라인 78,061픽셀 및 장애물 121,605픽셀. `experiments/dl/runs/{official-smoke2,random-smoke}/summary.json` 참조. 샘플 영상과 오버레이를 육안으로 비교했다.
- 대량 수집은 학습 2,100장, 검증 180장, 랜덤 시험 360장, 공식 시험 180장으로 배정했다. 이 수치는 계획치다.

## 2026-11-23 22:19 KST · 학습 및 ONNX 예비 검증

- `42d8f41`에 수집기, 소형 U-Net 학습·평가, `segment(bgr_img)` 예제를 커밋했다. 단위 테스트 4개를 통과했다.
- RTX 3090에서 별도 예비 실행으로 학습 420장과 검증 420장을 1 epoch 처리하고 ONNX 내보내기까지 확인했다. `experiments/dl/runs/training-smoke/training.json`에 기록했다. 이 모델은 최종 평가에 쓰지 않는다.
- 랩 컨테이너에서 ONNX Runtime 1.32.0으로 추론해 480×640 마스크 2개를 반환했다. 전체 수집·최종 학습과 평가가 진행 중이다.

## 2026-11-23 22:27 KST · 최종 수집·학습·평가

- 10개 맵 2,820장 수집 완료. 학습 2,100장(시드 101–105), 검증 180장(201), 시험 540장(301–303 및 공식 트랙). 모든 PNG·마스크·프레임 명세 수와 맵 분리를 확인했다. 원본은 `experiments/dl/runs/dataset-v1/`, 집계는 `experiments/dl/results/dataset.json`.
- RTX 3090에서 24 epoch. 검증 라인 IoU 0.9154, 장애물 0.9730. `experiments/dl/results/{training.json,epochs.jsonl}`.
- 홀드아웃 랜덤 360장에서 U-Net 라인 0.8573, 장애물 0.9629. OpenCV gray<80은 각각 0.3313, 0.0000. 공식 180장의 라인 IoU는 U-Net 0.8242, OpenCV 0.8745. 공식 트랙 15장에 장애물 오검출이 있었다. `experiments/dl/results/evaluation.json`.
- 전처리·마스크 생성 포함 CPU 257.3 FPS(p95 4.92 ms), RTX 3090 359.1 FPS(p95 3.58 ms). ONNX와 PyTorch 출력을 8장으로 대조했고 랩에서 기본 `segment()` 호출을 검증했다.
- 모델 `experiments/dl/line_seg.onnx`, 상세 표와 실패 사례 `experiments/dl/REPORT.md`, Codex-1 연동 예제 `experiments/dl/README.md`. 잔여 문제는 공식 트랙의 그림자 오검출과 먼 라인·프레임 가장자리 상자 누락이다. 완주율과 회피 성능은 추종기 연동 후 확인한다.
