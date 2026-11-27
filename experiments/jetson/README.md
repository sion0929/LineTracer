# Jetson Orin Nano 8GB 배포 계획

실물 로봇은 Jetson Orin Nano 8GB Developer Kit이다. 6코어 Cortex-A78AE CPU, 1024코어 Ampere GPU, 8 GB 공유 메모리를 쓴다. 채점은 외부 PC에서 하고, 그 PC에는 GPU가 없을 수 있다. 그래서 **같은 코드가 두 환경에서 다 돌아야 한다.**

| 환경 | 추론 | 비고 |
|---|---|---|
| Jetson | onnxruntime + TensorRT (FP16) | 첫 실행 때 엔진을 만들어 `trt_cache/`에 저장한다. 두 번째 실행부터 빠르다 |
| GPU 없는 채점 PC | onnxruntime CPU, 없으면 OpenCV DNN | `restore.py`가 이 순서로 자동 선택한다 |

## 시간 예산 (카메라 30 Hz → 프레임당 33 ms)

| 항목 | 호출 빈도 | x86 CPU 측정 (부하 약 50) | Jetson 목표 |
|---|---|---|---|
| v4 분할+깊이 기본형 (320×240) | 매 프레임 | ort 9.1 / p95 19.0 ms | TensorRT 5 ms 이하 |
| v4 분할+깊이 소형 | 매 프레임 | ort 5.5 / p95 11.4 ms | TensorRT 3 ms 이하 |
| 선 복원 R1 (128×128) | 경로가 끊길 때만 | ort 8.1 / p95 15.6 ms, cv2 24.5 ms | TensorRT 3 ms 이하 |
| 규칙 기반 선 마스크와 지도 갱신 | 매 프레임 | 약 1–3 ms | 확인 필요 |

## 보드에서 측정하기 (ROS 불필요)

```bash
# 이 폴더(experiments/jetson)를 보드로 복사한 뒤
pip3 install numpy onnx            # onnxruntime-gpu는 Jetson AI Lab 인덱스의 JetPack용 휠을 쓴다
python3 bench_models.py --runs 200 --threads 4
# -> bench_<호스트이름>.json 에 모델별·백엔드별 중앙값/p95/최대 지연이 남는다
sudo nvpmodel -q; sudo jetson_clocks --show   # 전원 모드도 함께 기록한다 (MAXN 권장)
```

## ROS 2 Jazzy 실행 환경

- JetPack 6은 Ubuntu 22.04 기반이다. Jazzy는 Ubuntu 24.04용이라 Docker로 띄운다(l4t 기반 Jazzy 이미지). 패키지 코드는 그대로 쓴다.
- 카메라는 MIPI CSI-2다. `/camera/image_raw` 이름으로 내보내면 노드가 바뀌지 않는다.

## 할 일

1. 보드에서 `bench_models.py` 결과 받기 (사용자)
2. 결과에 따라 모델 크기 정하기: 기본형 또는 소형, 입력 해상도 320×240 또는 256×192
3. 분할·깊이 추론 경로도 onnxruntime(TensorRT) 우선으로 통일하기. 지금은 `restore.py`만 적용돼 있다
4. Jazzy Docker 이미지에 onnxruntime-gpu 넣기
