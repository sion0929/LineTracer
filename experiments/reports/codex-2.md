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

## 2026-11-23 22:43 KST · v2·하이브리드·거리별 재현율

- DeepSeek 워커에 `LAB_NAME=lf_b ROS_DOMAIN_ID=12 GZ_PARTITION=lf_b`를 지정해 무장애물 학습 720장, 검증 120장 수집과 v2 학습·평가를 맡겼다. 수집·평가 결과 JSON을 직접 확인했다. 총 학습 2,820장, 검증 300장, 기존 시험 540장. 공식 프레임은 시험에만 있다. `experiments/dl/results/v2/dataset.json`.
- 조명 기울기와 쐐기형 그림자 증강, 640×480 RGB 경계 보정을 학습한 v2의 공식 라인 IoU는 **0.9423**으로 OpenCV 0.8745와 v1 0.8242보다 높았다. 공식 오검출은 15/180 → 14/180장, 최대 28,465 → 24,888픽셀.
- 랜덤 라인 IoU는 v1 DL 0.8573, v1 하이브리드 0.8987, v2 DL 0.9604, v2 하이브리드 0.7529. 랜덤 장애물 IoU는 v1 0.9629, v2 0.8810.
- 1–2m 공식 라인 픽셀 재현율은 59.5% → 89.9%. 0.5m 미만 랜덤 화면 가장자리 상자는 92.6% → 67.4%로 하락했다. 표는 `experiments/dl/REPORT.md`, 거리 구간별 분모와 분자는 `experiments/dl/results/v2/{v1,v2}.json`.
- CPU 46.8 FPS(p95 25.77ms), RTX 3090 149.8 FPS(p95 9.44ms). `experiments/dl/line_seg_v2.onnx`로 랩에서 `segment`와 `segment_hybrid` API 검증. 공식 그림자 오검출과 가까운 가장자리 상자 누락, 추종기 주행 결과가 잔여 확인 항목이다.

## 2026-11-23 22:58 KST · G3 CPU 후보

- `dev/two-track`의 `07f842f`를 받아 Codex-1 `364e419`와 원본 `experiments/follower.py`가 일치함을 확인했다. 복사본 `experiments/dl/follower_dl.py`에 v2 라인, v1 장애물, 좌우 여유와 지도 복귀를 추가했다(`9bddf25`).
- DeepSeek가 `lf_b`에서 `bench.py --backend obstacle --off 0.12`로 평가했다. CPU 후보는 공식 0/3, 고정 G2 시드 20·26·33에서 0/3. 충돌 0건, 회피 성공 0건. 공식 진행률 21.24% / 11.24% / 16.02%, G2 진행률 7.66% / 8.33% / 6.70%.
- 6회 모두 콜백 p95 63.16–100.99ms, 시뮬 영상 처리 13.55–22.31Hz로 30Hz 목표 미달. `experiments/dl/runs/follower-bench/dual-a/*/{result.json,run.log}`에서 확인했다. 랩의 CUDAExecutionProvider에서 두 ONNX를 읽고 추론을 확인한 뒤 같은 계획을 GPU 후보로 다시 위임했다.

## 2026-11-23 23:09 KST · G3 CUDA 후보 비교 수치

- 같은 주행 설정과 v2 라인·v1 장애물 ONNX를 RTX 3090에서 실행했다. DeepSeek의 6회 결과와 보충 1회 결과를 `result.json`과 `run.log`에서 직접 확인했다. 채점은 고정 G2와 `--backend obstacle --off 0.12`를 사용했다.
- 공식 3/3 완주, 랩타임 179.652 / 186.708 / 176.116초(중앙값 179.652초), 최대 횡오차 3.49 / 8.94 / 3.39cm, 충돌 0건. 바퀴 접촉 판정은 각 6회였다.
- G2 기본 시드 20·26·33은 4.502 / 4.801 / 4.790m에서 T자 종료 오인 후 정체했다. 보충 시드 21은 3.83m의 상자를 오른쪽 24cm로 우회하고 선 지도로 복귀했다. bench 회피 성공 1/1, 충돌 0건. 이후 T자 종료 오인으로 10.598m(70.53%)에서 정체했다. GPU G2 전체 0/4 완주, 충돌 0건, 회피 성공 1/1.
- CUDA 7회 모두 콜백 p95 18.61–28.09ms로 33.3ms 이내였다. 시뮬 영상 처리율은 29.16–30.25Hz였고 공식 두 회에서 30Hz에 조금 못 미쳤다. T자 종료 오인과 바퀴 접촉이 잔여 문제다.
- 비교표 `experiments/dl/REPORT.md`, 실행별 수치 `experiments/dl/results/g3/`, 원본 실행 `experiments/dl/runs/follower-bench/{dual-a,dual-cuda-b,dual-cuda-b-early}/`. 기준 추종기와 공용 채점기 소스는 병합본 `07f842f`를 사용했다.

## 2026-11-23 23:10 KST · 바퀴 접촉 지표 반영

- `git merge dev/two-track` 결과 이미 최신이었다. 바퀴 지표 `1f00892`와 고정 G2 세트 `076fe40`가 모두 현재 브랜치에 있다.
- 기존 13회 벤치의 두 접촉 열을 원본 `result.json`과 대조해 `experiments/dl/REPORT.md`에 추가했다. GPU 공식 3회는 각각 6회, 접촉 시간 1.856 / 1.320 / 1.560초다. G2 회피 성공 시드 21은 6회, 5.520초다.
- 반지름 10cm 코너의 안쪽 바퀴 접촉을 고려해 ±0.10m 통로 안의 경로 최적화 목표에 접촉 횟수와 시간을 포함했다. 기존 실측을 기준으로 사용한다.
