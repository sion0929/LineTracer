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

맵과 프로필의 모든 조합을 평가한다. 각 주행은 예약 범위 30–101에서 별도 ROS 도메인을 받고, Gazebo 파티션은 `lf_c_<배치ID>_<주행번호>`처럼 배치와 주행을 식별한다. 환경의 `ROS_DOMAIN_ID`는 예약 범위에서 제외하고, `GZ_PARTITION`은 파티션 접두사로 사용한다. `--jobs`는 동시 실행 수이며 매트릭스당 최대 주행 수는 72개다. 컨테이너에서 여러 벤치를 띄우면 도메인 잠금으로 같은 도메인의 사용 시간을 분리한다. 실제 할당은 `job.json`, `result.json`, 결과 CSV에서 확인할 수 있다. 명령은 빈 결과 디렉터리에서 시작한다.

- 기본 백엔드는 `--backend run_eval`이다. `run_eval.py`의 채점 함수에 맵의 중심선과 같은 슬롯의 Gazebo를 연결한다. 원래 기준에 따라 8 cm를 넘는 이탈이 0.6 s 지속되면 실패하고, 도착 12 cm 이내면 완료한다. 원본과 같은 벽시계 랩타임을 제공한다. `sim_sec`는 센서에서 수집한 별도 시뮬레이션 경과 시간이다.
- `--backend obstacle`은 상자 근처에 제한된 우회 구간을 허용하며 랩타임에 시뮬레이션 시간을 쓴다. 실험 스윕은 이 채점 방식을 사용한다. 두 방식 모두 상자 뒤에서 중심선에 복귀하고 접촉이 없었던 경우를 회피로 기록한다. 충돌 수는 접촉한 상자의 개수다.
- `results.csv`와 터미널 표를 생성한다. 각 실행의 `result.json`, `trajectory.npy`, `run.log`는 번호로 구분된 하위 디렉터리에 있다. 기본 백엔드는 `results/legacy.json`, `results/legacy.log`에 원본 채점 출력과 follower 로그도 보존한다.
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

`--trials`는 기존 완료 trial을 포함한 목표 유효 횟수다. 같은 맵 경로·내용, follower, 채점 코드, 시드와 시간 설정으로 같은 폴더를 넘기면 SQLite 기록에서 이어간다. 래퍼 추가 전의 SQLite는 기존 채점 코드 해시를 보존하므로 해당 코드를 가진 커밋에서 이어간다. 중단 시 남은 RUNNING trial은 FAIL로 정리한다. 시험 결과는 `study.sqlite3`, `config.json`, 자동 갱신되는 `summary.md`, 개별 `trial_NNNN/results.csv`에서 확인한다. 인프라 오류가 세 번 연속 발생하면 중단한다.

점수는 완주 수에 최대 0.99점의 속도·정밀도 보너스를 더한다. 같은 완주 수에서는 완주한 맵의 평균 랩타임에 0.79, 모든 맵의 평균 횡오차 p95에 0.20을 배정한다. 첫 trial은 follower 기본값으로 실행한다.

## 2026-11-23 결과 재현

기존 독립 채점 기록을 재현하는 절차다. 랩 컨테이너의 `/ws`에서 다음 시드와 상자 수로 생성한다.

```bash
python3 experiments/worlds/gen_world.py --seed 0 --difficulty easy --obstacles 0
python3 experiments/worlds/gen_world.py --seed 9 --difficulty normal --obstacles 1
python3 experiments/worlds/gen_world.py --seed 14 --difficulty hard --obstacles 2

LAB_NAME=lf_c ROS_DOMAIN_ID=13 GZ_PARTITION=lf_c \
  python3 experiments/bench.py src/line_follower_bringup/worlds/line_track.sdf \
  experiments/worlds/generated/{easy_0,normal_9,hard_14}.sdf \
  --backend obstacle --jobs 2 --limit 140 --output experiments/worlds/generated/baseline_repro

export PYTHONPATH=/ws/experiments/sweeps/.deps:${PYTHONPATH:-}
args=(experiments/worlds/generated/{easy_0,normal_9}.sdf --jobs 2 --limit 32 \
      --stall 8 --output experiments/sweeps/2026-11-23-repro)
LAB_NAME=lf_c ROS_DOMAIN_ID=13 GZ_PARTITION=lf_c python3 experiments/sweep.py "${args[@]}" --trials 3
LAB_NAME=lf_c ROS_DOMAIN_ID=13 GZ_PARTITION=lf_c python3 experiments/sweep.py "${args[@]}" --trials 10
```

스윕 검증용 32초 제한은 재개와 결과 저장을 확인하기 위한 설정이다. 전체 코스의 속도와 완주율에 맞춰 튜닝할 때는 충분한 `--limit`과 별도의 결과 폴더를 사용한다.

## 고정 20맵 기본 follower 비교

기본 명령은 공식 트랙 1개와 생성 맵 20개를 채점한다. 생성 맵만 채점하려면 아래 첫 줄을 `maps=()`로 시작한다. 시드와 상자 수를 [20맵 미리보기](../worlds/preview_20.png)와 동일하게 둔다. 랩 컨테이너에서 ROS Jazzy 환경을 소싱한 다음 저장소 루트 `/ws`에서 실행한다.

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
  python3 experiments/bench.py "${maps[@]}" --backend run_eval --jobs 3 --limit 140 \
  --output experiments/worlds/generated/bench20_repro
```

공식 트랙과 생성 맵을 비교할 때 follower의 커밋과 인자를 고정한다. 생성 맵 평균은 시드 0–19의 20개 주행을 집계한다. 결과 CSV의 `reason`이 `error:`로 시작하면 해당 실행은 인프라 오류로 분리한다.

이 측정의 follower를 이후 브랜치에서 재현하려면 호스트 워크트리에서 다음과 같이 고정한다. 컨테이너의 벤치 명령에 `--follower experiments/worlds/generated/baseline_follower.py`를 추가한다.

```bash
mkdir -p experiments/worlds/generated
git show b16a161:experiments/follower.py > experiments/worlds/generated/baseline_follower.py
```
