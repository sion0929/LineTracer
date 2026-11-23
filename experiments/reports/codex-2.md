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

## 2026-11-23 22:32 KST · 그림자 증강 후속 비교

- DeepSeek 워커를 `LAB_NAME=lf_b ROS_DOMAIN_ID=12 GZ_PARTITION=lf_b`로 백그라운드 실행했다. 24 epoch 후보 학습 1회와 기준·후보 평가 각 1회를 마쳤다. 결과 수치는 각 실행의 JSON에서 확인했다.
- 공식 180장의 장애물 오검출이 15장 → 8장, 한 프레임의 최대 오검출이 28,465 → 9,601픽셀로 감소했다. 공식 라인 IoU는 0.8242 → 0.8356.
- 랜덤 합계 라인 IoU는 0.8573 → 0.8671, 장애물 IoU는 0.9629 → 0.9550. Hard 303 장애물 IoU는 0.9375 → 0.9248. 후보 처리량은 CPU 187.3 FPS, GPU 238.7 FPS.
- 어려운 맵의 장애물 IoU를 기준으로 기본 ONNX는 기존 모델을 유지했다. 개선 후보의 ONNX는 이 워크트리의 `experiments/dl/runs/shadow-study/shadow/line_seg.onnx`에 있다. 수치 원본과 같은 프레임 비교 이미지는 `experiments/dl/results/shadow-study/`에 기록했다. 시험 세트 재평가이므로 새 맵과 추종기에서의 검증은 남아 있다.

## 2026-11-23 22:33 KST · 공용 생성기 병합

- 리드가 검증한 생성기 병합 `90a4c59`를 포함하는 `dev/two-track`을 `track/b-dlseg`에 병합했다(`585203a`). 수집 스크립트와 안내가 현재 워크트리의 `experiments/worlds/gen_world.py`를 사용한다.
- 수집 스크립트로 랜덤 맵 9개를 다시 생성했다. SDF·STL·JSON 27개 SHA256이 `experiments/dl/results/dataset.json`의 기록과 모두 일치했다. 기존 이미지·마스크는 수집 완료 기록을 사용했다. 막힌 점 없음.
