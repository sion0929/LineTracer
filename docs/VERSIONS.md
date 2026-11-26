# 인지 버전 이름표

이름 규칙은 `V<번호>-<부품><해상도>-<학습 데이터>`다. 파일 이름에서는 `&` 대신 `+`를 쓴다(셸에서 `&`가 깨진다). 옛 파일 이름은 같은 폴더에 링크로만 남아 있다.

## 인지 파이프라인 (팀에 넘기는 단위)

| 버전 | 가까운 곳 | 먼 곳 | 상태 |
|---|---|---|---|
| V0-NearRule640 | 규칙 기반 임계값, 640×480 원본 | 없음 (BEV 0.85 m) | 원래 제출 기준 |
| V1-Near320&Far640 | 규칙 기반, 320×240 | V2-Far640-Gz, 640 원본의 지평선 띠 | 듀얼 해상도. Isaac 공식 트랙 완주 68.6 s |
| V2-NearDL320&Far640 | V4-NearB320-Isaac (선 분할 + 깊이) | V4-Far640-Isaac | Isaac 재생 평가로 V1과 비교 중 |
| V2-NearDL320&Far640-TRT16 | 위와 같음, TensorRT FP16 엔진 | 위와 같음, TensorRT FP16 엔진 | Jetson 배포용 |

## 모델

### 가까운 곳: 선 분할 + 깊이 (입력 320×240)

| 이름 | 옛 이름 | 학습 데이터 | 검증 선 IoU |
|---|---|---|---|
| V0-NearS320-Gz | seg_depth_t4_small | Gazebo | – |
| V1-NearS320-Gz | seg_depth_v4_small | Gazebo | – |
| V1-NearB320-Gz | seg_depth_v4_base | Gazebo | – |
| V1-NearB320FarDepth-Gz | seg_depth_t1_far | Gazebo, 먼 깊이 가중 | – |
| V2-NearS320-Isaac | near_v4s_isaac | Isaac 1라운드 | – |
| V2-NearB320-Isaac | near_v4b_isaac | Isaac 1라운드 | – |
| V3-NearB320-Isaac | near_v4b_isaac_r2 | Isaac 2라운드 | 0.935 |
| V4-NearB320-Isaac | near_v4b_isaac_r3 | Isaac 3라운드 (맵 286개 + 기존) | 0.939 |

S는 small, B는 base다.

### 먼 곳: 지평선 띠 (640 원본에서 64행, 열은 절반으로 줄여 64×320 입력)

| 이름 | 옛 이름 | 학습 데이터 |
|---|---|---|
| V1-Far640-Gz | farband_w16 | Gazebo (폭 640 그대로) |
| V2-Far640-Gz | farband_w8h2 | Gazebo 평지 |
| V3-Far640Hill-Gz | farband_w8h2_hill | Gazebo 평지 + 언덕 |
| V4-Far640-Isaac | farband_isaac | Isaac 포함 (선택 지표 0.884) |
| V4-Far640Wide-Isaac | farband_isaac_w12 | Isaac 포함, 폭 12 (0.890, 느려서 미채택) |

### 선 복원 (로봇 주변 지도 128×128)

| 이름 | 옛 이름 |
|---|---|
| V1-Restore128 | restore_r1, r1_w16 |
| V2-Restore128 | restore_r2, r2_w24 |
| V3-Restore128 | restore_r3 |
| V4-Restore128 | restore_r4 |

## TensorRT 엔진

엔진은 장치마다 따로 만든다. Jetson에서 ONNX 옆에 같은 이름으로 만든다. 노드는 `.onnx` 경로를 받으면 옆의 `.engine`을 자동으로 쓴다(`trt_runner.resolve_model`, 끄려면 `LF_NO_TRT=1`).

```bash
# Jetson의 TensorRT 11에는 --fp16 플래그가 없다. FP16 그래프(입출력은 float32 유지)로 변환한 뒤 빌드한다.
trtexec --onnx=V4-NearB320-Isaac.fp16.onnx --saveEngine=V4-NearB320-Isaac.engine
```

PC 검증 결과는 다음과 같다(Isaac 검증 프레임 20장, OpenCV DNN 출력 대비).

| 모델 | FP32 엔진 | FP16 엔진 |
|---|---|---|
| V4-NearB320-Isaac | 마스크 IoU 0.9995, 깊이 차 0.002 cm | 마스크 IoU 0.9986, 깊이 차 0.012 cm |
| V4-Far640-Isaac | 마스크 IoU 0.9992 | 마스크 IoU 0.9988 |
