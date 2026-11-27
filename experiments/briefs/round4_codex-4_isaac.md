# Codex-4: Isaac Sim 인지 학습 환경 (d-isaac)

우리 팀은 **인지 파트만** 한다. 판단·제어는 기준 구현으로 고정한다. 너의 일은 Isaac Sim에서 선 분할과 깊이 학습용 합성 데이터를 **GPU 병렬로** 대량 생성하는 환경을 만드는 것이다. 모델 학습은 Codex-2(b-dlseg)가 맡는다. 너는 데이터와 환경을 만든다.

## 주어진 것
- Isaac Sim 5.1.0이 `/home/rvlab/isaacsim`에 설치돼 있다(`./python.sh`, `./isaac-sim.sh`). 새로 설치하지 말고 이걸 쓴다.
- GPU는 RTX 3090 24 GB다. **사용률 90%, VRAM 21 GB를 넘기지 않는다.** Codex-2 학습과 GPU를 같이 쓰니 `nvidia-smi`로 남은 양을 확인하고 맞춘다.
- 디스크 여유는 141 GB(92% 사용)다. 데이터셋은 50 GB를 넘기지 않는다. 이미지는 PNG가 아니라 JPEG(q90)나 npz로 압축한다.
- RAM은 31 GB이고 지금 여유가 적다. 헤드리스로 돌린다.
- 맵 생성기 `experiments/worlds/gen_world.py`와 각 맵의 `.json`(centerline, markings, ramps, obstacles, loop_corners, detours, gaps)이 있다. 이걸 Isaac 장면으로 옮긴다.
- 카메라 기준은 TurtleBot3 Waffle Pi다. 해상도, FOV, 장착 높이와 각도, 프레임 주기는 `src/line_follower_bringup/models/`의 SDF에서 읽는다. **원본 `worlds/`와 `models/`는 절대 수정하지 않는다**(SHA256 검사).

## 할 일
1. **I1 장면 변환:** gen_world JSON을 USD 장면으로 바꾸는 `experiments/isaac/json_to_usd.py`를 만든다. 바닥 매트, 선(폭 0.02 m), 언덕 경사, 상자 장애물이 들어간다. 공식 트랙(`line_track.sdf`)도 변환한다.
2. **I2 카메라와 라벨:** Waffle Pi 카메라와 같은 사양으로 RGB를 만든다. 라벨은 선 분할 마스크, 깊이, 카메라 자세, BEV 선 마스크다. BEV는 `vision.py`의 IPM과 같은 격자로 만든다.
3. **I3 병렬 생성:** Replicator나 Isaac Lab으로 환경을 여러 개 복제한다. 카메라 자세는 선 위 궤적을 따라가며 횡오차 ±0.12 m, 요 ±30°, 언덕 피치 범위로 뽑는다. 조명, 바닥 텍스처, 선 색과 밝기, 그림자, 노이즈, 블러를 무작위로 바꾼다(도메인 랜덤화). 처리량(장/초)과 GPU 사용률을 기록한다.
4. **I4 Gazebo와의 차이 확인:** 같은 맵, 같은 자세에서 Isaac 이미지와 Gazebo 이미지를 나란히 놓는다. Gazebo 이미지는 `experiments/` 데이터 수집 도구로 얻는다. Codex-2의 v3 모델로 두 쪽의 라인 IoU를 비교해 표로 남긴다.
5. **I5 인계:** 데이터셋 형식을 Codex-2 학습 코드가 바로 읽을 수 있게 맞춘다. 형식은 `b-dlseg` worktree의 `experiments/dl/`를 보고 정한다. 인계 문서는 `experiments/isaac/README.md`에 쓴다.

## 규칙
- Isaac 코드는 `experiments/isaac/` 안에만 둔다. **제출 패키지(`src/`)에는 넣지 않는다.**
- 오래 걸리는 작업은 `nohup`이나 `setsid`로 띄워 터미널이 닫혀도 살아 있게 한다. 로그는 `experiments/results/d-isaac/`에 남긴다.
- CPU 부하가 높다. Gazebo를 쓸 때는 동시 실행을 최대 2개로 하고, 측정할 때 `uptime` 부하를 같이 적는다.
- 커밋은 track/d-isaac 브랜치에만 한다. push는 하지 않는다.
- 단계마다 끝나면 `experiments/results/d-isaac/REPORT.md`에 수치(처리량, VRAM, IoU 차이)를 추가한다.

## 합격
- I3: GPU 병렬로 **초당 100장 이상**을 VRAM 21 GB 안에서 만든다.
- I4: Isaac으로 학습한 모델이 Gazebo 화면에서 라인 IoU 0.90 이상을 내는 데이터 설정을 찾는다. Isaac 데이터와 Gazebo 데이터를 섞은 설정도 함께 비교한다.
