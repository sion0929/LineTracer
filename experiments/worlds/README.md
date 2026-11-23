# 랜덤 라인트레이서 월드

저장소 루트에서 생성한다. 같은 시드·난이도·상자 수와 파일명을 사용하면 SDF, STL, JSON이 바이트 단위로 재현된다.

```bash
python3 experiments/worlds/gen_world.py --seed 3 --difficulty normal --obstacles 2
python3 experiments/worlds/gen_world.py --seed 9 --difficulty hard --obstacles 3 \
  --output experiments/worlds/generated/training_9.sdf --preview
```

기본 경로는 `experiments/worlds/generated/{difficulty}_{seed}.sdf`다. SDF, STL, JSON을 함께 옮기면 메시의 상대 경로가 유지된다. `--preview`에는 Matplotlib이 필요하다. 상자는 기본 0개이고, 하나 이상 배치하면 첫 상자가 중심선을 막는다. 코너, 경사, 교차부 부근은 상자 배치 대상에서 제외한다.

| 난이도 | 원호 반지름 | 직각 코너 | 경사 높이 | 중심선과 교차하는 루프 |
| --- | --- | ---: | --- | ---: |
| easy | 0.25–0.34 m | 1 | 2.5–3.8 cm | 1 |
| normal | 0.14–0.27 m | 2 | 4.0–6.0 cm | 1 |
| hard | 0.08–0.17 m | 3 | 5.8–7.5 cm | 2 |

공식 매트 색과 폭 20 mm, 지표면 위 1 mm의 검은 STL 선을 사용한다. 정답 경로는 S자형이고 루프는 경로와 교차하는 보조선이다. [20종 미리보기](preview_20.png)는 시드 0–19, 상자 수 `seed % 4`, 난이도 easy 7개·normal 7개·hard 6개를 담는다.

## Gazebo 실행

`lf_c` 랩 컨테이너에서 다음처럼 실행한다. 다른 워크트리는 자기 랩 이름, ROS 도메인과 Gazebo 파티션을 사용한다.

```bash
sg docker -c 'LAB_NAME=lf_c ROS_DOMAIN_ID=13 GZ_PARTITION=lf_c tools/lab_env.sh up'
sg docker -c 'LAB_NAME=lf_c ROS_DOMAIN_ID=13 GZ_PARTITION=lf_c docker exec \
  -e LAB_NAME=lf_c -e ROS_DOMAIN_ID=13 -e GZ_PARTITION=lf_c lf_c bash -c "
    source /opt/ros/jazzy/setup.bash
    export GZ_SIM_RESOURCE_PATH=/ws/src/line_follower_bringup/models
    gz sim -s -r --headless-rendering /ws/experiments/worlds/generated/normal_3.sdf
  "'
```

TurtleBot3는 월드에 포함된다. Gazebo 월드 이름은 기존 캡처 도구와 호환되는 `line_track`이다. 위 명령은 Gazebo 서버만 띄우며 ROS 브리지는 별도로 실행한다.

## JSON 형식

- `centerline`은 출발부에서 도착부까지 진행 순서대로 정렬된 `[x, y, z]` 배열이다. `z`는 표면 높이이며 실제 검은 선은 여기에 `line_lift_m`를 더한 높이에 있다. `arc_m`은 각 점의 누적 평면 거리다.
- `markings`는 출발·도착 T자 가로선과 보조 루프를 나타내는 폴리라인이다. 카메라의 라인 마스크를 생성할 때는 이 배열과 `centerline`을 함께 투영한다.
- `start_pose` 및 각 상자의 `pose`는 `[x, y, z, roll, pitch, yaw]` 순서다. 단위는 m와 rad다. 상자 좌표는 중심점이며 `size`는 상자 자체 좌표계에서의 XYZ 크기다. `on_line`은 상자 중심의 중심선 배치 여부다.
- `ramp.x`는 오르막 시작, 평탄부 시작·끝, 내리막 끝의 X 좌표다. `ramp.y`는 경사 표면의 Y 범위, `ramp.height_m`은 최대 높이다.

## G2 상자 장애물 평가 세트

[eval_set_G2.txt](eval_set_G2.txt)는 시드 20–39의 고정 평가 맵 20개를 실행 순서대로 담는다. 파일명에 난이도, 시드, 상자 수가 들어 있다. 생성기 버전은 `532e0c8`이며 난이도는 easy 6개, normal 7개, hard 7개다. 각 맵에 상자 1–3개가 있으며 최소 한 상자가 중심선 위에 있다. G2 점수의 재현을 위해 평가 세트는 파라미터 탐색용 맵과 분리한다.

저장소 루트에서 목록에 지정된 파일을 생성한다. 각 SDF 옆에 같은 이름의 STL과 JSON이 함께 저장된다.

```bash
while IFS= read -r map; do
  case "$map" in ''|\#*) continue ;; esac
  file=${map##*/}
  IFS=_ read -r difficulty seed obstacles <<< "${file%.sdf}"
  python3 experiments/worlds/gen_world.py --seed "$seed" --difficulty "$difficulty" \
    --obstacles "${obstacles#obs}" --output "$map" || break
done < experiments/worlds/eval_set_G2.txt
```

생성한 맵을 랩 컨테이너의 `/ws`에서 평가할 때는 아래처럼 목록 순서대로 전달한다. 장애물 인근의 우회와 복귀를 평가하는 백엔드는 `obstacle`이다.

```bash
mapfile -t g2_maps < <(sed '/^#/d; /^$/d' experiments/worlds/eval_set_G2.txt)
LAB_NAME=lf_c ROS_DOMAIN_ID=13 GZ_PARTITION=lf_c \
  python3 experiments/bench.py "${g2_maps[@]}" --backend obstacle --jobs 3 \
    --output experiments/worlds/generated/g2_bench
```
