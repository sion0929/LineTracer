# 병렬 벤치마크와 Optuna 스윕

## 벤치마크

랩 컨테이너에서 ROS Jazzy 환경을 소싱한 후 실행한다. 공식 트랙은 저장소 안의 원본 SDF 경로를 넘긴다. 생성 맵은 SDF 옆에 STL과 JSON을 둔다.

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
LAB_NAME=lf_c ROS_DOMAIN_ID=13 GZ_PARTITION=lf_c \
  python3 experiments/bench.py \
  src/line_follower_bringup/worlds/line_track.sdf \
  experiments/worlds/generated/normal_3.sdf \
  --profile 'default=' --profile 'slow=--vmax 0.16' \
  --jobs 2 --output experiments/worlds/generated/bench_example
```

맵과 프로필의 모든 조합을 평가한다. 첫 실행 슬롯은 지정된 ROS 도메인과 Gazebo 파티션을 쓴다. 다음 슬롯은 예약 범위 31–46의 별도 도메인과 관리 파티션을 쓴다. 각 평가에 자체 시뮬레이터와 ROS 브리지가 있다. 명령은 빈 결과 디렉터리에서 시작한다. 표의 `reason` 필드에서 이탈, 정체, 시뮬레이터 오류 등의 종료 원인을 확인할 수 있다.

- `results.csv`와 터미널 표를 생성한다. 각 실행의 `result.json`, `trajectory.npy`, `run.log`는 번호로 구분된 하위 디렉터리에 있다.
- 기존 기준에 따라 8 cm를 넘는 이탈이 0.6 s 지속되면 실패하고, 도착 12 cm 이내면 완료한다. 시간과 랩타임은 시뮬레이션 시간으로 계산한다. 랩타임은 2 cm 진행부터 완료까지다. 횡오차는 순서가 있는 중심선의 지역 투영을 사용한다.
- 중심선 위 상자 근처에는 제한된 우회 구간을 허용한다. 옆으로 이탈하고 상자 뒤의 중심선에 다시 붙어야 회피로 기록하며, Gazebo 접촉 센서가 해당 상자의 충돌을 감지하면 회피 수에서 제외한다. 충돌 수는 접촉한 상자의 개수다.
- 슬립은 휠 오도메트리와 Gazebo 자세의 시뮬레이션 시간이 근접한 0.25 s 구간을 비교한다. 선속도 차가 0.10 m/s 이상이거나 회전속도 차가 0.65 rad/s 이상인 연속 구간이 한 이벤트다. `odom_windows`와 `slip_events`에 유효 구간 수와 이벤트별 수치를 저장한다.

## 스윕

랩 Python에서 Optuna 4.x를 사용할 수 있어야 한다. 같은 Ubuntu Python 3.12의 호스트에서 설치 가능한 예시는 다음과 같다.

```bash
/usr/bin/python3 -m pip install --target experiments/sweeps/.deps 'optuna>=4,<5' 'numpy<2'
```

랩 컨테이너에서는 다음처럼 설치 위치를 전달한다. 이미 Optuna가 설치돼 있으면 `PYTHONPATH` 인자를 생략한다.

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
LAB_NAME=lf_c ROS_DOMAIN_ID=13 GZ_PARTITION=lf_c \
PYTHONPATH=/ws/experiments/sweeps/.deps:${PYTHONPATH:-} \
  python3 experiments/sweep.py \
  experiments/worlds/generated/easy_0.sdf \
  experiments/worlds/generated/normal_3.sdf \
  --jobs 2 --trials 10 --output experiments/sweeps/2026-11-23-full
```

`--trials`는 기존 완료 trial을 포함한 목표 유효 횟수다. 같은 맵 경로·내용, follower, 채점 코드, 시드와 시간 설정으로 같은 폴더를 넘기면 SQLite 기록에서 이어간다. 중단 시 남은 RUNNING trial은 FAIL로 정리한다. 시험 결과는 `study.sqlite3`, `config.json`, 자동 갱신되는 `summary.md`, 개별 `trial_NNNN/results.csv`에서 확인한다. 인프라 오류가 세 번 연속 발생하면 중단한다.

점수는 완주 수에 최대 0.99점의 속도·정밀도 보너스를 더한다. 같은 완주 수에서는 완주한 맵의 평균 랩타임에 0.79, 모든 맵의 평균 횡오차 p95에 0.20을 배정한다. 첫 trial은 follower 기본값으로 실행한다.

## 2026-11-23 결과 재현

랩 컨테이너의 `/ws`에서 다음 시드와 상자 수로 생성한다.

```bash
python3 experiments/worlds/gen_world.py --seed 0 --difficulty easy --obstacles 0
python3 experiments/worlds/gen_world.py --seed 9 --difficulty normal --obstacles 1
python3 experiments/worlds/gen_world.py --seed 14 --difficulty hard --obstacles 2

LAB_NAME=lf_c ROS_DOMAIN_ID=13 GZ_PARTITION=lf_c \
  python3 experiments/bench.py src/line_follower_bringup/worlds/line_track.sdf \
  experiments/worlds/generated/{easy_0,normal_9,hard_14}.sdf \
  --jobs 2 --limit 140 --output experiments/worlds/generated/baseline_repro

export PYTHONPATH=/ws/experiments/sweeps/.deps:${PYTHONPATH:-}
args=(experiments/worlds/generated/{easy_0,normal_9}.sdf --jobs 2 --limit 32 \
      --stall 8 --output experiments/sweeps/2026-11-23-repro)
LAB_NAME=lf_c ROS_DOMAIN_ID=13 GZ_PARTITION=lf_c python3 experiments/sweep.py "${args[@]}" --trials 3
LAB_NAME=lf_c ROS_DOMAIN_ID=13 GZ_PARTITION=lf_c python3 experiments/sweep.py "${args[@]}" --trials 10
```

스윕 검증용 32초 제한은 재개와 결과 저장을 확인하기 위한 설정이다. 전체 코스의 속도와 완주율에 맞춰 튜닝할 때는 충분한 `--limit`과 별도의 결과 폴더를 사용한다.

## 고정 20맵 기본 follower 비교

시드와 상자 수를 [20맵 미리보기](../worlds/preview_20.png)와 동일하게 둔다. 랩 컨테이너에서 ROS Jazzy 환경을 소싱한 다음 저장소 루트 `/ws`에서 실행한다.

```bash
maps=(src/line_follower_bringup/worlds/line_track.sdf)
for seed in {0..19}; do
  if (( seed < 7 )); then difficulty=easy
  elif (( seed < 14 )); then difficulty=normal
  else difficulty=hard
  fi
  map="experiments/worlds/generated/${difficulty}_${seed}.sdf"
  python3 experiments/worlds/gen_world.py --seed "$seed" --difficulty "$difficulty" \
    --obstacles "$((seed % 4))" --output "$map"
  maps+=("$map")
done

LAB_NAME=lf_c ROS_DOMAIN_ID=13 GZ_PARTITION=lf_c \
  python3 experiments/bench.py "${maps[@]}" --jobs 3 --limit 140 \
  --output experiments/worlds/generated/bench20_repro
```

공식 트랙과 생성 맵을 비교할 때 follower의 커밋과 인자를 고정한다. 생성 맵 평균은 시드 0–19의 20개 주행을 집계한다. 결과 CSV의 `reason`이 `error:`로 시작하면 해당 실행은 인프라 오류로 분리한다.

이 측정의 follower를 이후 브랜치에서 재현하려면 호스트 워크트리에서 다음과 같이 고정한다. 컨테이너의 벤치 명령에 `--follower experiments/worlds/generated/baseline_follower.py`를 추가한다.

```bash
mkdir -p experiments/worlds/generated
git show b16a161:experiments/follower.py > experiments/worlds/generated/baseline_follower.py
```
