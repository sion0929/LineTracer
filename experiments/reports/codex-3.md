## 2026-11-23 · 단계 1 · 랜덤 월드

- A·B용 생성기와 JSON 형식을 `experiments/worlds/README.md`에 공개했다. 중심선·경사 높이·T자/교차 루프·상자 자세를 제공한다.
- 시드 0–19를 두 번 생성해 SDF/STL/JSON이 모두 일치했다. 중심선 길이는 14.74–16.42 m, 평균 15.67 m였다. 공식 중심선은 15.70 m다. 그림은 `experiments/worlds/preview_20.png`에 저장했다.
- Gazebo Harmonic에서 normal 시드 3에 상자 2개를 넣어 스폰을 확인했다. 첫 자세의 XY 오차는 4.35×10⁻¹⁰ m였고, 카메라 프레임은 640×480이었다. 전방 ROI의 최소 픽셀 밝기는 0이었다. 측정 파일은 `experiments/results/C_map_spawn_normal3.json`이다.
- 공식 월드·모델 SHA256 12개가 모두 일치했다. 난이도는 공식 맵과 길이·경사·코너 규모를 맞춰 설계했으며, 주행 결과로 난이도 분포를 확인하는 작업은 벤치마크 단계에서 이어간다.

## 2026-11-23 · 단계 2·4 · 병렬 채점·슬립

- `experiments/bench.py`는 맵 × follower 프로필을 독립 Gazebo/ROS 슬롯에서 실행하고, 순서 중심선 투영과 시뮬레이션 타임스탬프로 진행률·완주·랩타임·횡오차를 계산한다. 상자 센서로 접촉을 집계하고, 오도메트리와 실제 자세의 선속도·회전속도 차이로 슬립을 기록한다. 사용법은 `experiments/sweeps/README.md`에 있다.
- ROS 도메인 13·31/Gazebo 파티션 `lf_c`·`lf_c_bench_31`에서 기본 follower로 4개 맵을 주행했다. 완주 0/4, 평균 진행률 17.35%, 슬립 4건이었다. 기본 주행은 상자에 도달하기 전에 종료됐다. CSV는 `experiments/results/C_bench_defaults.csv`, 실행별 로그와 궤적은 `experiments/worlds/generated/baseline/`에 있다.
- 접촉 검증용 맵에서 직진 명령을 주었을 때 상자 접촉 1건과 슬립 1건을 기록했다. 슬립의 최대 속도 차는 0.198 m/s였다. 측정 파일은 `experiments/results/C_bench_contact.json`이다. 우회 후 중심선 복귀 판정은 합성 궤적에서 검증했다.
- 기본 follower는 공식·hard 맵에서 이탈, easy·normal 맵에서 정체로 종료했다. 실제 상자 회피율은 아직 측정하지 못했다. 벤치와 스윕 실행은 Gazebo EGL 경고가 없는지와 무관하게 결과의 인프라 오류 여부로 구별한다.

## 2026-11-23 · 단계 3 · Optuna 스윕

- `experiments/sweep.py`에 6개 파라미터 탐색, 완주 우선 점수, SQLite 재개, trial별 CSV와 자동 요약을 추가했다. 같은 DB에서 목표를 3회에서 10회로 올리니 기존 trial을 유지하고 7회만 추가 실행했다. 별도 SQLite 복사본에서는 중단 상태의 trial만 FAIL로 바뀌고 기존 점수가 유지됐다.
- easy 시드 0과 normal 시드 9로 유효 trial 10/10, 실제 주행 20회, 완주 0/20, 정체 17회와 이탈 3회였다. 최고 점수는 trial 8의 0.1470, 평균 진행률은 17.64%, 평균 횡오차 p95는 0.0212 m였다. 실행 제한 32초와 정체 제한 8초를 사용한 재개·기록 검증이므로 완주 성능과 실전 파라미터는 검증이 남았다.
- SQLite와 개별 주행은 `experiments/sweeps/2026-11-23/`에 있다. 커밋에 포함한 요약은 `experiments/results/C_sweep_smoke_summary.md`, 맵별 로그는 `experiments/results/DS-C_20261123-2220_sweep.log`다. 재현 명령은 `experiments/sweeps/README.md`에 정리했다.
- 랩 컨테이너 전체 테스트는 205개, 오류·실패·건너뜀 각 0개였다. 로그는 `experiments/worlds/generated/final_colcon_test.log`다.
