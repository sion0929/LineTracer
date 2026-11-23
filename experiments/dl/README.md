# 라인·장애물 분할

TurtleBot3 전방 카메라의 BGR 프레임에서 라인과 상자의 이진 마스크를 얻는다. 수집에는 Gazebo의 카메라와 월드 자세를 쓰며, 런타임 입력은 영상 한 장이다.

## 수집

`b-dlseg` 워크트리 루트에서 랩을 띄운다.

```bash
export LAB_NAME=lf_b ROS_DOMAIN_ID=12 GZ_PARTITION=lf_b
sg docker -c 'tools/lab_env.sh build'
bash experiments/dl/collect_dataset.sh
```

데이터 경로는 `experiments/dl/runs/dataset-v1`이다. 맵마다 `image/`, `mask/` 아래에 640×480 PNG가 쌓인다. 마스크 값은 배경 0, 선 1, 장애물 2다. `frames.jsonl`에는 실제 카메라 프레임 시각과 Gazebo 자세, 표본의 횡편차·각도, 클래스별 픽셀 수가 담긴다. `summary.json`은 수집이 끝난 맵에만 작성된다. 재실행하면 완료한 맵을 건너뛴다.

워크트리에 병합된 `experiments/worlds/gen_world.py`가 생성한 버전 1 SDF/JSON/STL 묶음을 사용한다. 중심선을 따라 위치를 뽑고, 횡방향 ±0.1 m와 진행 방향 ±40° 범위에서 자세를 바꾼다. 상자 앞쪽도 추가로 샘플링한다. 라인 STL과 지면·상자 기하를 실제 로봇 자세로 투영하고 깊이 순서로 가림을 처리한다. 공식 트랙은 시험에만 포함한다.

| 용도 | 맵 시드 | 목표 프레임 |
| --- | --- | ---: |
| 학습 | 101–105 | 2,100 |
| 검증 | 201 | 180 |
| 시험 | 301–303 및 공식 트랙 | 540 |

한 맵만 수집하려면 해당 SDF와 같은 폴더에 JSON·STL을 두고 `--world`를 지정한다. 공식 트랙에서는 `--world`를 생략한다. 카메라 한 장을 수집하지 못하면 콘솔과 출력 디렉터리의 `gazebo.log`를 확인한다.

```bash
sg docker -c "docker exec -e LAB_NAME -e ROS_DOMAIN_ID -e GZ_PARTITION lf_b bash -c \
  'source /opt/ros/jazzy/setup.bash && python3 -m experiments.dl.collect \
  --out experiments/dl/runs/official-check --count 16 --seed 11'"
```

## 학습 및 평가

호스트의 `lf_b_dl` Python 환경을 사용한다. 이 환경은 Miniforge `lerobot`의 CUDA PyTorch와 OpenCV를 공유하는 venv다. 새 환경의 필수 패키지는 CUDA를 지원하는 `torch`, `numpy`, `opencv-python-headless`, `onnx`, `onnxruntime`이다.

```bash
DL_PY=/home/rvlab/miniforge3/envs/lf_b_dl/bin/python
DL_DATA=experiments/dl/runs/dataset-v1
DL_RUN=experiments/dl/runs/small-unet-v1
"$DL_PY" -m experiments.dl.train --data "$DL_DATA" --out "$DL_RUN"
"$DL_PY" -m experiments.dl.evaluate --data "$DL_DATA" \
  --onnx "$DL_RUN/line_seg.onnx" --checkpoint "$DL_RUN/best.pt" \
  --out experiments/dl/runs/eval-v1 --benchmark-frames 480
```

학습은 320×240 RGB 입력에 밝기, 대비, 감마, 채널별 색상 이득, 좌우반전 증강을 적용한다. 검증 맵의 라인·장애물 평균 IoU가 가장 높은 체크포인트를 ONNX로 내보낸다. 평가 IoU는 원본 640×480의 기하 라벨을 기준으로 계산한다. OpenCV 비교는 BGR을 회색조로 바꾼 후 밝기 80 미만을 라인으로 분류한다. CPU에서 ONNX Runtime으로, GPU에서 같은 체크포인트를 PyTorch로 실행하며 두 측정 모두 영상 전처리와 최종 마스크를 포함한다.

`train`의 `--shadow-prob 0.5` 옵션은 학습 프레임에 부드러운 경계의 국소 그림자를 추가한다. `evaluate`는 맵별로 정답 장애물이 없는 프레임 수, 그중 장애물 픽셀을 하나 이상 예측한 프레임 수와 프레임당 최대 오검출 픽셀 수를 `obstacle_audit`에 기록한다.

### v2 재현

v1 데이터와 학습 체크포인트를 먼저 만든 뒤 무장애물 맵을 추가한다. 랜덤 학습 시드 401–403과 검증 시드 451을 수집하며 기존 시험 세트를 연결한다.

```bash
bash experiments/dl/collect_v2.sh
DL_DATA=experiments/dl/runs/dataset-v2
DL_V2=experiments/dl/runs/v2-study/v2
"$DL_PY" -m experiments.dl.train --data "$DL_DATA" --out "$DL_V2" \
  --epochs 24 --batch-size 12 --seed 12 --shadow-prob 0.40 \
  --official-style --full-resolution --learning-rate 0.0007 \
  --initialize-from experiments/dl/runs/small-unet-v1/best.pt
"$DL_PY" -m experiments.dl.evaluate --data "$DL_DATA" \
  --onnx "$DL_V2/line_seg.onnx" --checkpoint "$DL_V2/best.pt" \
  --out experiments/dl/runs/v2-study/v2-eval --distance-recall --benchmark-frames 480
```

거리별 재현율은 정답 픽셀을 카메라 광축의 전방 거리로 나눈다. 라인은 전 화면, 가장자리 상자는 화면의 좌우 64픽셀과 상하 48픽셀 띠를 사용한다.

## Codex-1 연동

저장소 루트를 Python 모듈 검색 경로에 추가한다. ROS 이미지의 `encoding`을 확인해 RGB 영상은 BGR로 변환하고, `step`에 패딩이 있으면 제거한다. 기본 모델은 `experiments/dl/line_seg.onnx`이며 `LINE_SEG_ONNX`로 경로를 바꿀 수 있다. 랩에서 실행할 때는 랩 Python이 `onnxruntime`을 찾을 수 있어야 한다.

```python
from experiments.dl.infer import segment, segment_hybrid

# follower.py의 --perception dl 및 --perception hybrid 분기에서 호출한다.
perceive = segment_hybrid if args.perception == "hybrid" else segment
line_mask, obstacle_mask = perceive(bgr_img)
dark = line_mask[RAY_V, RAY_U].ravel()[ok0]
```

하이브리드 라인은 회색조 밝기 80 미만인 픽셀에서 DL 장애물 마스크를 뺀 결과다. 장애물 마스크는 DL 출력이다.

랩 이미지에 pip가 없는 경우, Python 3.12 호스트 환경에서 아래 명령을 한 번 실행한다. 랩에서 추종기를 실행할 때 `PYTHONPATH`에 `/ws/experiments/dl/runs/lab-runtime:/ws`를 추가한다.

```bash
/home/rvlab/miniforge3/envs/lf_b_dl/bin/python -m pip install --no-deps \
  --target experiments/dl/runs/lab-runtime onnxruntime==1.32.0 flatbuffers packaging
```

두 마스크는 모두 `(480, 640)`의 `numpy.bool_` 배열이다. 첫 호출에 ONNX 세션을 로드하므로 주행 전에 한 번 호출해 준비할 수 있다. 장애물 마스크는 장애물 회피 로직 입력으로 사용한다.

## G3 비교용 추종기

`follower_dl.py`는 Codex-1의 `364e419`를 복사해 카메라 인식과 장애물 우회를 붙인 별도 실행 파일이다. 라인은 v2, 장애물은 v1을 기본 모델로 쓴다. 장애물로 가려진 픽셀은 라인 지도의 음성 증거에서 빼고, 일반 추종은 지도의 선 중심을 목표로 한다.

일반 추종의 설계 통로는 선 중심 기준 ±0.10m다. 좁은 통로를 막은 상자는 카메라 마스크에서 직립 영역의 하단을 투영해 위치를 추정한다. 좌우 여유를 로봇 폭과 비교한 뒤, 가능한 쪽으로 비켜 지나가고 기존 라인 지도에 복귀한다. 원래 통로 밖 이동은 우회 구간에서 허용한다. `bench.py --backend obstacle --off 0.12`가 일반 이탈, 상자 접촉과 복귀, 바퀴 접촉을 채점한다.

```bash
bash experiments/dl/prepare_lab_cuda.sh
DL_RUNTIME_DIR=/ws/experiments/dl/runs/lab-runtime-cuda \
DL_CUDA_LIBS=/ws/experiments/dl/runs/lab-cuda-libs \
bash experiments/dl/bench_follower.sh dual-cuda-b --device cuda --ort-threads 1
```

CUDA 실행에는 RTX 3090에 접근 가능한 `lf_b` 랩 빌드와 CUDA 13·cuDNN 9 라이브러리가 필요하다. 준비 스크립트는 기존 호스트 라이브러리를 워크트리의 실행 폴더에 연결한다. 이름마다 별도 디렉터리에 공식 3회와 고정 G2의 20·26·33번 각 1회가 저장된다. `DL_BENCH_JOBS=1`로 동시 주행 수를 줄일 수 있다. 실행별 `result.json`과 `run.log`에 채점 및 추론·콜백 타이밍이 기록된다.
