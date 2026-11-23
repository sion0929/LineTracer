# 라인·장애물 분할

TurtleBot3 전방 카메라의 BGR 프레임에서 라인과 상자의 이진 마스크를 얻는다. 수집에는 Gazebo의 카메라와 월드 자세를 쓰며, 런타임 입력은 영상 한 장이다.

## 수집

`b-dlseg` 워크트리 루트에서 랩을 띄운다.

```bash
export LAB_NAME=lf_b ROS_DOMAIN_ID=12 GZ_PARTITION=lf_b
sg docker -c 'tools/lab_env.sh build'
bash experiments/dl/collect_dataset.sh \
  /home/rvlab/orca/workspaces/26_robotprogramming_project/c-bench/experiments/worlds/gen_world.py
```

데이터 경로는 `experiments/dl/runs/dataset-v1`이다. 맵마다 `image/`, `mask/` 아래에 640×480 PNG가 쌓인다. 마스크 값은 배경 0, 선 1, 장애물 2다. `frames.jsonl`에는 실제 카메라 프레임 시각과 Gazebo 자세, 표본의 횡편차·각도, 클래스별 픽셀 수가 담긴다. `summary.json`은 수집이 끝난 맵에만 작성된다. 재실행하면 완료한 맵을 건너뛴다.

Codex-3의 버전 1 SDF/JSON/STL 묶음을 사용한다. 중심선을 따라 위치를 뽑고, 횡방향 ±0.1 m와 진행 방향 ±40° 범위에서 자세를 바꾼다. 상자 앞쪽도 추가로 샘플링한다. 라인 STL과 지면·상자 기하를 실제 로봇 자세로 투영하고 깊이 순서로 가림을 처리한다. 공식 트랙은 시험에만 포함한다.

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

## Codex-1 연동

저장소 루트를 Python 모듈 검색 경로에 추가한다. ROS 이미지의 `encoding`을 확인해 RGB 영상은 BGR로 변환하고, `step`에 패딩이 있으면 제거한다. 기본 모델은 `experiments/dl/line_seg.onnx`이며 `LINE_SEG_ONNX`로 경로를 바꿀 수 있다. 랩에서 실행할 때는 랩 Python이 `onnxruntime`을 찾을 수 있어야 한다.

```python
from experiments.dl.infer import segment

# follower.py의 --perception dl 분기에서 호출한다.
line_mask, obstacle_mask = segment(bgr_img)
dark = line_mask[RAY_V, RAY_U].ravel()[ok0]
```

랩 이미지에 pip가 없는 경우, Python 3.12 호스트 환경에서 아래 명령을 한 번 실행한다. 랩에서 추종기를 실행할 때 `PYTHONPATH`에 `/ws/experiments/dl/runs/lab-runtime:/ws`를 추가한다.

```bash
/home/rvlab/miniforge3/envs/lf_b_dl/bin/python -m pip install --no-deps \
  --target experiments/dl/runs/lab-runtime onnxruntime==1.32.0 flatbuffers packaging
```

두 마스크는 모두 `(480, 640)`의 `numpy.bool_` 배열이다. 첫 호출에 ONNX 세션을 로드하므로 주행 전에 한 번 호출해 준비할 수 있다. 장애물 마스크는 장애물 회피 로직 입력으로 사용한다.
