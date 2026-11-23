# Codex-2 라인·장애물 분할 결과

2026-11-23 KST · Gazebo Harmonic, TurtleBot3 Waffle Pi

## 데이터와 모델

| 데이터 | 맵 | 640×480 영상 |
| --- | --- | ---: |
| 학습 | 랜덤 시드 101–105 | 2,100 |
| 검증 | 랜덤 시드 201 | 180 |
| 시험 | 랜덤 시드 301–303 | 360 |
| 시험 | 공식 트랙 | 180 |
| 합계 | 10개 맵 | 2,820 |

Codex-3 생성기 `532e0c8`의 중심선을 따라 위치를 샘플링했다. 수집 범위는 횡편차 ±0.1 m와 방향 편차 ±40°이며 언덕 위와 상자 앞의 자세를 포함한다. Gazebo 자세와 카메라 영상의 시각을 맞춘 뒤 라인 메시와 상자, 경사와 지면을 투영했다. 깊이 비교로 상자가 가린 라인을 제거했다. 맵별 표본 범위와 픽셀 수, 생성한 파일의 SHA256은 [데이터 집계](results/dataset.json)에 있다. 원본 프레임과 마스크는 이 워크트리의 `experiments/dl/runs/dataset-v1/`에 있다.

소형 U-Net은 파라미터 106,323개를 사용한다. 320×240 RGB 입력에 밝기, 대비, 감마, 색상 이득과 좌우반전을 적용하고 RTX 3090에서 24 epoch 학습했다. 학습 및 내보내기에 50.7초가 걸렸다. 검증 맵에서 라인 IoU 0.9154, 장애물 IoU 0.9730을 기록한 24번째 epoch를 선택했다. [학습 집계](results/training.json)와 [epoch 기록](results/epochs.jsonl).

## 홀드아웃 IoU

같은 640×480 기하 마스크를 두 방식에 적용했다. OpenCV는 회색조 밝기 80 미만을 라인으로, 나머지를 배경으로 분류한다. 랜덤 맵 합계는 각 맵의 혼동행렬을 합산해 계산했다. 공식 트랙은 학습과 검증에 포함하지 않았다.

| 시험 맵 | U-Net 배경 | OpenCV 배경 | U-Net 라인 | OpenCV 라인 | U-Net 장애물 | OpenCV 장애물 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 공식 트랙 | 0.9966 | 0.9982 | 0.8242 | **0.8745** | 0.0000 | — |
| Easy 301 | 0.9975 | 0.9941 | **0.8591** | 0.3288 | **0.9802** | 0.0000 |
| Normal 302 | 0.9968 | 0.9824 | **0.8571** | 0.3625 | **0.9800** | 0.0000 |
| Hard 303 | 0.9944 | 0.9866 | **0.8561** | 0.3099 | **0.9375** | 0.0000 |
| 랜덤 맵 합계 | 0.9963 | 0.9877 | **0.8573** | 0.3313 | **0.9629** | 0.0000 |

공식 트랙에는 정답 장애물이 없다. U-Net은 180장 중 15장에서 장애물 픽셀을 검출해 IoU가 0이다. OpenCV 장애물 열의 대시는 정답과 예측의 합집합이 비어 있음을 뜻한다. 랜덤 맵에서는 OpenCV가 어두운 상자 면을 라인으로 분류했다. 클래스별 교차와 합집합을 재계산할 수 있는 혼동행렬은 [평가 원본](results/evaluation.json)에 있다.

## 추론 속도

40회 워밍업 후 시험 프레임으로 480회 측정했다. 카메라 프레임의 전처리, 모델 실행, 640×480 불리언 마스크 2개 생성을 포함했다. GPU 수치에는 입력과 출력의 호스트 전송도 포함한다.

| 장치와 백엔드 | 평균 | p95 | 처리량 |
| --- | ---: | ---: | ---: |
| Intel Core i7-10700 · ONNX Runtime 1.32.0, 스레드 4개 | 3.89 ms | 4.92 ms | **257.3 FPS** |
| NVIDIA RTX 3090 · PyTorch 2.11.0+cu130 | 2.78 ms | 3.58 ms | **359.1 FPS** |

두 장치에서 목표 30 FPS를 넘었다. [ONNX 모델](line_seg.onnx)은 opset 17이며 424 KiB다. ONNX 검사와 실제 프레임 8장의 PyTorch 출력 비교를 통과했다. `lf_b` 컨테이너에서 기본 모델을 사용하는 `segment(bgr_img)` 호출로 480×640 불리언 마스크 2개를 확인했다. Codex-1용 연동 예제는 [README](README.md#codex-1-연동)에 있다. 측정 범위는 프레임 단위 분할이며, 실제 회피 동작과 주행 완주율은 연동 후 확인할 항목이다.

## 실패 예시

왼쪽부터 카메라 영상, 기하 정답, U-Net, OpenCV 결과다. 초록은 라인, 주황은 장애물이다.

- [공식 000094](results/failure-line_track-line.jpg)에서 먼 라인이 배경과 섞이고 언덕 그림자 경계 일부가 라인으로 검출됐다. 이 프레임의 U-Net 라인 IoU는 0.251이었다.
- [공식 000177](results/failure-line_track-obstacle-fp.jpg)에서 언덕 주변 그림자 28,465픽셀이 장애물로 분류됐다.
- [Hard 303 000089](results/failure-hard_303-line.jpg)에서 멀리 있는 보조선 일부를 놓쳤다. 이 프레임의 U-Net 라인 IoU는 0.359였다.
- [Hard 303 000082](results/failure-hard_303-obstacle.jpg)에서 왼쪽 화면 경계에 걸친 상자 일부를 놓쳤다. 이 프레임의 U-Net 장애물 IoU는 0.510이었다.

## 후속 실험 · 국소 그림자 증강

같은 학습·검증 분할에서 `--shadow-prob 0.5`로 24 epoch를 학습하고 기존 시험 540장을 다시 평가했다. 선택 기준은 검증 맵의 전경 평균 IoU였다. 증강 모델의 검증 전경 평균 IoU는 0.9358이었다.

| 지표 | 기준 U-Net | 그림자 증강 U-Net |
| --- | ---: | ---: |
| 공식 라인 IoU | 0.8242 | 0.8356 |
| 공식 장애물 오검출 프레임 | 15/180 | 8/180 |
| 공식 프레임당 최대 장애물 오검출 | 28,465픽셀 | 9,601픽셀 |
| 랜덤 합계 라인 IoU | 0.8573 | 0.8671 |
| 랜덤 합계 장애물 IoU | 0.9629 | 0.9550 |
| Hard 303 장애물 IoU | 0.9375 | 0.9248 |

그림자 증강으로 공식 오검출 프레임이 7장 줄었다. 기본 ONNX는 Hard 303 장애물 IoU가 더 높은 기준 모델로 유지했다. 이 비교는 동일 시험 세트의 재평가다. [공식 000177 비교 이미지](results/shadow-study/official-000177.jpg) · [기준 평가](results/shadow-study/baseline.json) · [후보 평가](results/shadow-study/shadow.json) · [후보 학습](results/shadow-study/training.json)

## v2 · 공식 트랙 라인과 하이브리드 비교

병합된 생성기로 무장애물 학습 맵 401–403에서 720장, 검증 맵 451에서 120장을 추가했다. 학습 2,820장과 검증 300장은 모두 랜덤 맵이다. 시험은 기존과 같은 공식 180장과 미사용 랜덤 360장이다. [데이터 집계](results/v2/dataset.json).

v1 가중치에서 시작해 640×480 입력과 RGB 경계 보정층을 사용했다. 조명 기울기와 경계 두 개로 만든 쐐기형 그림자를 증강했다. 24 epoch 중 검증 전경 평균 IoU가 가장 높은 20번째 epoch를 선택했다. [학습 집계](results/v2/training.json).

하이브리드는 OpenCV 회색조 밝기 80 미만의 라인 후보에서 해당 DL 모델의 장애물 마스크를 뺀다. 하이브리드의 장애물 마스크는 해당 DL 모델과 같다. 시험 세트의 픽셀 혼동행렬을 합산해 IoU를 계산했다.

| 시험 맵 | 클래스 | OpenCV | v1 DL | v1 하이브리드 | v2 DL | v2 하이브리드 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 공식 180장 | 배경 | 0.9982 | 0.9966 | 0.9977 | 0.9985 | 0.9975 |
| 공식 180장 | 라인 | 0.8745 | 0.8242 | 0.8734 | 0.9423 | 0.8745 |
| 공식 180장 | 장애물 | — | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| Easy 301 | 배경 | 0.9941 | 0.9975 | 0.9983 | 0.9992 | 0.9986 |
| Easy 301 | 라인 | 0.3288 | 0.8591 | 0.8992 | 0.9578 | 0.8976 |
| Easy 301 | 장애물 | 0.0000 | 0.9802 | 0.9802 | 0.9895 | 0.9895 |
| Normal 302 | 배경 | 0.9824 | 0.9968 | 0.9978 | 0.9925 | 0.9964 |
| Normal 302 | 라인 | 0.3625 | 0.8571 | 0.8999 | 0.9620 | 0.6860 |
| Normal 302 | 장애물 | 0.0000 | 0.9800 | 0.9800 | 0.8227 | 0.8227 |
| Hard 303 | 배경 | 0.9866 | 0.9944 | 0.9955 | 0.9932 | 0.9963 |
| Hard 303 | 라인 | 0.3099 | 0.8561 | 0.8974 | 0.9610 | 0.7268 |
| Hard 303 | 장애물 | 0.0000 | 0.9375 | 0.9375 | 0.8654 | 0.8654 |
| 랜덤 합계 360장 | 배경 | 0.9877 | 0.9963 | 0.9972 | 0.9950 | 0.9971 |
| 랜덤 합계 360장 | 라인 | 0.3313 | 0.8573 | 0.8987 | 0.9604 | 0.7529 |
| 랜덤 합계 360장 | 장애물 | 0.0000 | 0.9629 | 0.9629 | 0.8810 | 0.8810 |

### 거리별 픽셀 재현율

| 정답 픽셀 | 거리 | 지원 픽셀 | OpenCV | v1 DL | v1 하이브리드 | v2 DL | v2 하이브리드 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 공식 라인 | <0.5 m | 555,289 | 96.2% | 98.4% | 96.0% | 99.5% | 96.2% |
| 공식 라인 | 0.5–<1 m | 154,832 | 76.4% | 90.0% | 76.4% | 96.2% | 76.4% |
| 공식 라인 | 1–<2 m | 45,341 | 31.6% | 59.5% | 31.6% | 89.9% | 31.6% |
| 공식 라인 | ≥2 m | 3,276 | 2.7% | 17.1% | 2.7% | 32.4% | 2.7% |
| 랜덤 라인 | <0.5 m | 1,258,231 | 96.4% | 98.7% | 96.4% | 99.7% | 96.4% |
| 랜덤 라인 | 0.5–<1 m | 239,975 | 80.5% | 91.8% | 80.4% | 98.1% | 80.4% |
| 랜덤 라인 | 1–<2 m | 84,946 | 35.6% | 58.6% | 35.3% | 91.7% | 35.4% |
| 랜덤 라인 | ≥2 m | 8,102 | 3.7% | 15.4% | 3.7% | 32.7% | 3.7% |
| 랜덤 가장자리 상자 | <0.5 m | 629,578 | 0.0% | 92.6% | 92.6% | 67.4% | 67.4% |
| 랜덤 가장자리 상자 | 0.5–<1 m | 216,238 | 0.0% | 98.1% | 98.1% | 96.7% | 96.7% |
| 랜덤 가장자리 상자 | 1–<2 m | 91,053 | 0.0% | 96.6% | 96.6% | 97.3% | 97.3% |
| 랜덤 가장자리 상자 | ≥2 m | 9,568 | 0.0% | 96.0% | 96.0% | 93.3% | 93.3% |

거리는 라벨을 실제 Gazebo 카메라 자세로 재투영해 얻은 광축 방향 깊이다. 각 구간의 정답 픽셀 가운데 같은 클래스로 검출된 비율을 썼다. 가장자리 상자는 좌우 64픽셀과 상하 48픽셀 영역 안에 있는 상자 픽셀이다. 공식 시험에는 정답 상자가 없다.

v2 순수 DL은 공식 라인 IoU **0.9423**으로 OpenCV **0.8745**를 넘었다. 공식 장애물 오검출은 15/180장에서 14/180장으로, 한 프레임 최대 오검출은 28,465픽셀에서 24,888픽셀로 줄었다. 랜덤 상자 IoU는 0.9629에서 0.8810으로 내려갔고, 0.5m 미만 화면 가장자리 상자 재현율은 92.6%에서 67.4%로 내려갔다. v2 하이브리드는 어두운 상자 픽셀 중 DL이 놓친 픽셀을 다시 라인 후보로 분류해 랜덤 라인 IoU가 0.7529였다. [공식 라인 누락](results/v2/failure-line_track-line.jpg) · [Hard 303 가장자리 상자 누락](results/v2/failure-hard_303-obstacle.jpg).

v2 처리량은 i7-10700의 ONNX Runtime에서 **46.8 FPS**(p95 25.77ms), RTX 3090의 PyTorch에서 **149.8 FPS**(p95 9.44ms)였다. 40회 워밍업과 480회 측정에 전처리와 마스크 생성을 포함했다. [v1 결과 JSON](results/v2/v1.json) · [v2 결과 JSON](results/v2/v2.json).

[ONNX v2](line_seg_v2.onnx)는 `LINE_SEG_ONNX`로 선택한다. [Codex-1 연동 예제](README.md#codex-1-연동)의 `segment`와 `segment_hybrid`를 랩 컨테이너에서 각각 확인했다.

## G3 · 딥러닝 추종기 예비 비교

Codex-1 기준 `364e419`를 복사하고 v2 라인, v1 장애물 마스크와 카메라 기반 우회를 추가했다. 기본 지도 추종의 설계 통로는 ±0.10m다. 두 후보는 동일한 속도·조향 설정과 고정 G2 시드 20·26·33으로 비교했다. 채점은 `bench.py --backend obstacle --off 0.12`를 사용했다.

| 후보 | 공식 완주 | 공식 랩타임 | G2 완주 | G2 충돌 | G2 회피 성공 | 콜백 p95 | 시뮬 영상 처리율 |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 두 ONNX · CPU | 0/3 | — | 0/3 | 0 | 0 | 63.16–100.99ms | 13.55–22.31Hz |
| 두 ONNX · RTX 3090 | **3/3** | 179.652 / 186.708 / 176.116초 | 0/3 | 0 | 0 | **18.61–28.09ms** | 29.16–30.25Hz |

GPU의 공식 평균 랩타임은 180.825초, 중앙값은 179.652초다. 공식 최대 횡오차는 각각 3.49 / 8.94 / 3.39cm로 설계 통로 안이었다. GPU 콜백 p95는 모든 주행에서 33.3ms 안이었다. 두 공식 주행의 처리율은 각각 29.16 및 29.24Hz였다.

GPU 고정 시드 20·26·33의 진행률은 29.72 / 29.92 / 29.83%였다. 세 주행에서 상자 회피 판정에 앞서 로그에 `terminal T reached; stopping`이 기록됐다. 회피 시도가 0회인 이 세 주행으로 회피 성능을 판단할 수 없다. [CPU 실행별 원본 수치](results/g3/dual-a.json) · [GPU 실행별 원본 수치](results/g3/dual-cuda-b.json).

같은 GPU 후보로 첫 상자가 3.83m에 있는 고정 시드 21을 보충 평가했다. 오른쪽 24cm 우회 뒤 지도에 복귀했고, bench가 **회피 1/1, 충돌 0건**을 기록했다. 이후 잘못된 T자 종료 판정으로 10.598m(70.53%)에서 정체했다. 콜백 p95는 19.83ms, 영상 처리율은 30.23Hz였다. 보충을 포함한 GPU G2 실측은 **완주 0/4, 충돌 0건, 회피 1/1**이다. [시드 21 원본 수치와 추종기 로그 사건](results/g3/dual-cuda-b-early.json).

### 실행별 바퀴 선 접촉

`wheel_line_events`는 한쪽이라도 바퀴가 선을 밟기 시작한 횟수이고, `wheel_line_sec`는 바퀴가 선을 밟은 누적 시간(초)이다.

| 후보 | 맵 / 실행 | 진행률 | wheel_line_events | wheel_line_sec |
| --- | --- | ---: | ---: | ---: |
| CPU | 공식 1 | 21.24% | 1 | 0.792 |
| CPU | 공식 2 | 11.24% | 9 | 7.948 |
| CPU | 공식 3 | 16.02% | 8 | 5.096 |
| CPU | easy_20_obs1 | 7.66% | 2 | 0.876 |
| CPU | normal_26_obs1 | 8.33% | 2 | 1.644 |
| CPU | hard_33_obs2 | 6.70% | 1 | 0.180 |
| RTX 3090 | 공식 1 | 99.24% | 6 | 1.856 |
| RTX 3090 | 공식 2 | 99.24% | 6 | 1.320 |
| RTX 3090 | 공식 3 | 99.24% | 6 | 1.560 |
| RTX 3090 | easy_20_obs1 | 29.72% | 4 | 2.996 |
| RTX 3090 | normal_26_obs1 | 29.92% | 0 | 0.000 |
| RTX 3090 | hard_33_obs2 | 29.83% | 0 | 0.000 |
| RTX 3090 | easy_21_obs2 | 70.53% | 6 | 5.520 |

반지름 10cm 코너에서는 차체가 선 중심을 따라가도 안쪽 바퀴가 선을 밟을 수 있다. 통로 최적 경로의 목표에 선 중심 기준 ±0.10m 안에서 좌우 바퀴 궤적의 접촉 횟수와 시간을 함께 줄이는 것을 포함한다. 같은 맵에서 완주 여부와 진행 거리, 랩타임, 충돌·회피 지표와 함께 비교한다.

## G1 코너 재현 및 종료된 검증

규칙 기준판 `364e419`를 공식 3회 재현했다. 리드의 실패 최대 도달은 3.338m였고, 재현의 같은 두 번째 코너 실패는 3.294m였다. 두 주행 모두 코너에서 약 0.045m/s 직진 명령이 이어지는 동안 횡오차가 커졌다. [리드 실패와 재현 비교 PNG](results/g1/lead-vs-repro.png) · [리드 텔레메트리](../results/lead/g1_364e419_fail/telemetry.json).

`df30dec`의 딥러닝 급커브·통로 후보는 2/5 완주했고 총 4회가 3.34m를 통과했다. 한 회는 0.473m에서 이탈했고 영상 처리율이 46.10Hz로 기록돼 원인을 확정하지 못했다. 나머지 두 실패는 11.165m와 13.113m 정체였다. 리드 지시에 따라 대기 중이던 추가 코너 배치는 취소했다. 공통 코너 수정은 Codex-1이 맡는다.

| 배치 / 실행 | 완주 | 최대 s | 랩타임 | v p95 | 절대 a p95 | 절대 w p95 | 최대 횡오차 | ±10cm 밖 | 바퀴 접촉 횟수 / 시간 | 텔레메트리 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| g1-rule-repro / 01_01 | off line at 2.46 m | 2.458 m | — | 0.1000 m/s | 0.0418 m/s² | 0.3617 rad/s | 14.44 cm | 2.22 s | 2회 / 6.24 s | [JSON](runs/follower-bench/g1-rule-repro/01_01/telemetry.json) · [PNG](runs/follower-bench/g1-rule-repro/01_01/telemetry.png) |
| g1-rule-repro / 02_01 | 완주 | 15.584 m | 180.020 s | 0.1009 m/s | 0.0633 m/s² | 0.7969 rad/s | 3.35 cm | 0.00 s | 6회 / 1.86 s | [JSON](runs/follower-bench/g1-rule-repro/02_01/telemetry.json) · [PNG](runs/follower-bench/g1-rule-repro/02_01/telemetry.png) |
| g1-rule-repro / 03_01 | off line at 3.26 m | 3.294 m | — | 0.1003 m/s | 0.0554 m/s² | 0.5522 rad/s | 12.69 cm | 1.20 s | 1회 / 0.84 s | [JSON](runs/follower-bench/g1-rule-repro/03_01/telemetry.json) · [PNG](runs/follower-bench/g1-rule-repro/03_01/telemetry.png) |
| g1-dl-corridor / 01_01 | off line at 0.47 m | 0.473 m | — | 0.0340 m/s | 0.0379 m/s² | 0.3509 rad/s | 12.26 cm | 19.11 s | 4회 / 6.83 s | [JSON](runs/follower-bench/g1-dl-corridor/01_01/telemetry.json) · [PNG](runs/follower-bench/g1-dl-corridor/01_01/telemetry.png) |
| g1-dl-corridor / 02_01 | 완주 | 15.585 m | 205.448 s | 0.1005 m/s | 0.0583 m/s² | 0.7273 rad/s | 3.57 cm | 0.00 s | 6회 / 2.88 s | [JSON](runs/follower-bench/g1-dl-corridor/02_01/telemetry.json) · [PNG](runs/follower-bench/g1-dl-corridor/02_01/telemetry.png) |
| g1-dl-corridor / 03_01 | 완주 | 15.582 m | 203.532 s | 0.1006 m/s | 0.0558 m/s² | 0.7398 rad/s | 3.20 cm | 0.00 s | 6회 / 2.46 s | [JSON](runs/follower-bench/g1-dl-corridor/03_01/telemetry.json) · [PNG](runs/follower-bench/g1-dl-corridor/03_01/telemetry.png) |
| g1-dl-corridor / 04_01 | stalled at 11.16 m | 11.165 m | — | 0.1006 m/s | 0.0575 m/s² | 0.7238 rad/s | 3.42 cm | 0.00 s | 2회 / 1.02 s | [JSON](runs/follower-bench/g1-dl-corridor/04_01/telemetry.json) · [PNG](runs/follower-bench/g1-dl-corridor/04_01/telemetry.png) |
| g1-dl-corridor / 05_01 | stalled at 13.11 m | 13.113 m | — | 0.1005 m/s | 0.0603 m/s² | 0.7309 rad/s | 3.36 cm | 0.00 s | 5회 / 2.00 s | [JSON](runs/follower-bench/g1-dl-corridor/05_01/telemetry.json) · [PNG](runs/follower-bench/g1-dl-corridor/05_01/telemetry.png) |

## 고정 G2 장애물 세트 · 20맵

`e98ec64`의 RTX 3090 후보로 `eval_set_G2.txt` 20맵을 각각 한 번 평가했다. 파일럿 4맵과 나머지 16맵은 같은 `--tee-vision`, `--backend obstacle --off 0.12` 설정을 사용했다. T자 종료에는 짧은 가로팔 양쪽의 지지 증거가 필요하며, 전방에 선이 이어질 경우 그 선을 따라간다. 공식 코너 제어 옵션은 이 G2 후보에서 끈 상태로 평가했다.

| 난이도 | 완주 | 평균 최대 진행률 | 완주 랩타임 | 충돌 | 회피 성공/시도 | 바퀴 접촉 횟수 / 시간 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Easy | 1/6 | 61.86% | 195.744s | 1 | 2/3 | 29회 / 23.844s |
| Normal | 1/7 | 41.16% | 199.652s | 0 | 3/3 | 25회 / 19.852s |
| Hard | 0/7 | 47.72% | — | 2 | 2/4 | 11회 / 9.908s |
| 전체 | **2/20** | **49.66%** | 평균 197.698s | **3** | **7/10** | 65회 / 53.604s |

조기 T자 종료 로그는 0/20이었다. 기존에 4.5–4.8m에서 T자로 멈춘 시드 20·26·33은 각각 7.310m, 5.854m, 4.947m까지 갔고 이후 이탈했다. 새 후보의 콜백 p95는 19.88–31.21ms, 영상 처리율은 28.04–30.23Hz였다.

접촉을 검출한 세 주행은 회피 성공에서 제외했다. 시드 25에서 박스와 차체의 기하 여유가 가장 작았던 시각은 복귀 회전 구간이었다. 시드 35에서는 상자 앞에서 옆으로 비키는 중 여유가 가장 작았다. 추종기 자체가 복귀를 기록하더라도 벤치 접촉이 우선한다. [시드 24 완주와 시드 25 접촉의 텔레메트리 비교](results/g2/avoidance-comparison.png) · [20맵 요약 JSON](results/g2/fixed-set.json).

아래 표의 최대 진행, 횡오차 및 바퀴 접촉은 주행별 `telemetry.json`을 사용한다. 난이도별 바퀴 합계는 벤치 결과 `wheel_line_events`와 `wheel_line_sec`를 합산했다.

| 맵 | 최대 s | 완주 / 랩 | 충돌 | 회피 성공/시도 | 횡오차 p95 / 최대 | 바퀴 접촉 | 종료 | 텔레메트리 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| easy_20_obs1 | 7.310m | — | 0 | 0/0 | 5.9/14.5cm | 5회/4.07s | off line at 7.31 m | [JSON](runs/follower-bench/g2-dl-tee-pilot/01_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-pilot/01_01/telemetry.png) |
| easy_21_obs2 | 12.998m | — | 0 | 1/1 | 20.7/24.9cm | 7회/5.71s | off line at 13.00 m | [JSON](runs/follower-bench/g2-dl-tee-pilot/02_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-pilot/02_01/telemetry.png) |
| easy_22_obs3 | 3.406m | — | 0 | 0/0 | 0.7/0.9cm | 6회/4.96s | stalled at 3.40 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/01_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/01_01/telemetry.png) |
| easy_23_obs1 | 9.411m | — | 0 | 0/0 | 1.6/4.1cm | 3회/3.65s | stalled at 9.41 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/02_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/02_01/telemetry.png) |
| easy_24_obs2 | 15.704m | 195.744s | 0 | 1/1 | 20.6/26.1cm | 3회/2.04s | finished | [JSON](runs/follower-bench/g2-dl-tee-remaining/03_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/03_01/telemetry.png) |
| easy_25_obs3 | 8.125m | — | 1 | 0/1 | 26.7/28.5cm | 5회/5.16s | off line at 8.13 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/04_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/04_01/telemetry.png) |
| normal_26_obs1 | 5.854m | — | 0 | 0/0 | 3.2/14.2cm | 1회/1.38s | off line at 5.85 m | [JSON](runs/follower-bench/g2-dl-tee-pilot/03_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-pilot/03_01/telemetry.png) |
| normal_27_obs2 | 9.197m | — | 0 | 1/1 | 24.5/25.1cm | 3회/2.34s | off line at 9.20 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/05_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/05_01/telemetry.png) |
| normal_28_obs3 | 2.882m | — | 0 | 0/0 | 5.1/5.1cm | 4회/3.30s | stalled at 2.88 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/06_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/06_01/telemetry.png) |
| normal_29_obs1 | 2.984m | — | 0 | 0/0 | 8.9/9.0cm | 4회/3.66s | stalled at 2.98 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/07_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/07_01/telemetry.png) |
| normal_30_obs2 | 3.581m | — | 0 | 0/0 | 1.6/2.6cm | 8회/6.66s | stalled at 3.58 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/08_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/08_01/telemetry.png) |
| normal_31_obs3 | 5.188m | — | 0 | 1/1 | 26.5/27.6cm | 3회/2.52s | off line at 5.19 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/09_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/09_01/telemetry.png) |
| normal_32_obs1 | 15.875m | 199.652s | 0 | 1/1 | 19.1/25.1cm | 2회/1.50s | finished | [JSON](runs/follower-bench/g2-dl-tee-remaining/10_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/10_01/telemetry.png) |
| hard_33_obs2 | 4.947m | — | 0 | 0/0 | 4.5/15.7cm | 1회/0.30s | off line at 4.95 m | [JSON](runs/follower-bench/g2-dl-tee-pilot/04_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-pilot/04_01/telemetry.png) |
| hard_34_obs3 | 9.050m | — | 0 | 0/0 | 2.7/12.2cm | 1회/1.74s | off line at 9.05 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/11_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/11_01/telemetry.png) |
| hard_35_obs1 | 3.706m | — | 1 | 0/1 | 21.7/22.4cm | 1회/1.86s | stalled at 3.70 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/12_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/12_01/telemetry.png) |
| hard_36_obs2 | 3.842m | — | 1 | 0/1 | 19.4/20.1cm | 1회/0.78s | stalled at 3.84 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/13_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/13_01/telemetry.png) |
| hard_37_obs3 | 14.896m | — | 0 | 2/2 | 26.8/28.0cm | 5회/3.91s | off line at 14.90 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/14_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/14_01/telemetry.png) |
| hard_38_obs1 | 7.410m | — | 0 | 0/0 | 2.5/13.4cm | 1회/1.02s | off line at 7.41 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/15_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/15_01/telemetry.png) |
| hard_39_obs2 | 9.469m | — | 0 | 0/0 | 3.4/14.4cm | 1회/0.96s | off line at 9.47 m | [JSON](runs/follower-bench/g2-dl-tee-remaining/16_01/telemetry.json) · [PNG](runs/follower-bench/g2-dl-tee-remaining/16_01/telemetry.png) |
