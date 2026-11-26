# 맵 목록 (팀 공용)

2026-11-27 · 인지 파트

평가와 학습에 쓰는 맵을 한곳에 정리했다. 큰 세트는 git에 넣지 않고 **생성 명령으로 다시 만든다.** 생성기는 결정적이라 같은 명령이면 같은 파일이 나온다. decoy1000은 2026-11-27에 바이트 단위로 같음을 확인했다. 다른 세트는 그 뒤 생성기가 바뀌었을 수 있으니, 다시 만든 뒤 `index.csv`를 git에 있는 것과 비교한다.

맵 하나는 `<이름>.json`(중심선, 언덕, 칠한 선, 시작 자세 = 정답 기하), `<이름>.sdf`(Gazebo 월드), `<이름>.stl`(선 메시) 세 파일이다. Isaac은 JSON을 바로 읽는다(`run_isaac_drive.sh --map X.json`). Gazebo는 SDF를 쓴다.

## 1. 공식 트랙 (시험 전용)

| 항목 | 값 |
|---|---|
| 파일 | `src/line_follower_bringup/worlds/line_track.sdf` (**수정 금지**, CI가 SHA256 검사) |
| 길이 | 약 15.6 m, 22 cm 타일 15×15 |
| 언덕 | 1개. 오르막 0.44 m, 정상 평지 0.44 m, 내리막 0.44 m, 높이 6 cm (경사 약 7.8°) |
| 채점 구간 | 14구간 `experiments/eval/sections.json`, 이름은 `sections.py`. 공식 채점에서 실패한 구간은 4, 11, 14 |
| Isaac에서 | `--map official` |

공식 트랙으로 튜닝하지 않는다. 설정을 고른 뒤 마지막 확인에만 쓴다.

## 2. 세트 목록

| 세트 | 맵 수 | git | 만드는 법 | 내용 | 용도 |
|---|---|---|---|---|---|
| **decoy1000** | 1000 | 안 넣음 (147 MB) | `bash experiments/worlds/make_decoy1000.sh` | 공식 규격 타일 맵. ㄱ자·1/4원 코너, 교차, **언덕 840개**, 가짜 길(막다른 교차·곁가지) 평균 1.6개, 가짜 길 없는 맵 80개 | 학습·검증·시험 (아래 분할) |
| decoy_sample | 10 | 미리보기만 | `python3 experiments/worlds/gen_tiles.py --n 10 --seed 101 --decoys 3 --prefix decoy --out experiments/worlds/suites/decoy_sample --preview` | decoy1000 견본 | 빠른 확인 |
| tiles60 | 60 | 미리보기만 | `python3 experiments/worlds/gen_tiles.py` (기본값) | 가짜 길 없는 타일 맵 | 튜닝 |
| hills60 | 60 | 미리보기만 | `python3 experiments/worlds/make_suite.py --name hills60 --count 60 --seed0 9000 --hills 2-4 --loop-corners 0-1 --detours 0 --stubs 0 --crosses 0 --gaps 0` | 언덕 2–4개 | 언덕 평가 (Gazebo) |
| general200 | 200 | 미리보기만 | `TEAM_GUIDE.md` 4-1장 | 모든 요소 무작위 | 튜닝 |
| official_like50 | 50 | 미리보기만 | `TEAM_GUIDE.md` 4-1장 | 공식 트랙과 비슷한 난이도 | 튜닝 |
| stress50 | 50 | 미리보기만 | `TEAM_GUIDE.md` 4-1장 | 모든 요소 최대 | 스트레스 |
| obstacles100 | 100 | 미리보기만 | `TEAM_GUIDE.md` 4-1장 | 상자 1–3개 | **범위 밖** (장애물 없음 가정) |
| cases | 334 | 목록만 | `python3 experiments/worlds/gen_cases.py --out experiments/worlds/suites/cases --preview` | 상황 하나씩 (코너, 교차, 언덕 두 개 등), 좌우 반전 쌍 | 실패 상황 분석 |
| rand_sample | 10 | 넣음 | `experiments/worlds/suites/rand_sample/README.md` | decoy_0000의 조명·재질 무작위화 (Gazebo) | 무작위화 견본 |

"미리보기만"은 `index.csv`, `list.txt`, 미리보기 PNG만 git에 있고 맵 파일은 생성 명령으로 만든다는 뜻이다.

## 3. decoy1000 분할 (고정)

맵 번호로 나눈다. 바꾸지 않는다.

| 분할 | 번호 | 폴더 | 쓰는 곳 |
|---|---|---|---|
| 학습 | 0000–0899 | `experiments/worlds/suites/decoy1000_train/maps` | 모델 학습, 설정 튜닝, 판단 팀용 인지 로그(0000–0099) |
| 검증 | 0900–0949 | `.../decoy1000_val/maps` | 설정 비교 (예: 언덕 시야 실험은 검증 맵 중 언덕 있는 15개 + 공식 트랙) |
| 시험 | 0950–0999 | `.../decoy1000_test/maps` | 최종 확인만. 튜닝 금지 |

분할 폴더는 `decoy1000/part*/maps/`를 가리키는 상대 링크다. `make_decoy1000.sh`가 같이 만든다. 맵별 요소는 `decoy1000/part*/index.csv`(길이, 타일 수, 코너, 원호, 교차, 언덕 유무, 가짜 길 수)에 있다.

## 4. 언덕 기하 (decoy1000, 공식 트랙 공통)

JSON의 `ramps` 항목: `x0, y0, heading`(오르막 시작점과 방향), `up, deck, down`(각 0.44 m), `height_m`(0.06), 폭 0.66 m. 공식 트랙은 옛 형식 `x = [0.66, 1.10, 1.54, 1.98], y = [0, 0.66]`이다. 높이 계산은 `experiments/isaac/drive_core.py`의 `ground_height()`를 본다. 정상 평지가 0.44 m라서, 평지 시작점에서 카메라가 보는 0.31–0.85 m 가운데 0.44 m 너머는 이미 내리막이다.

## 5. Isaac 데이터 (git 밖)

Isaac 렌더링 프레임(학습용 npz, 맵당 40장)은 인지 PC의 로컬 디스크에만 있다. 필요하면 인지 파트에 요청한다. 같은 맵 JSON으로 `experiments/isaac/collect_decoy.py`를 돌리면 다시 만들 수 있다(GPU 필요, 수 시간).

## 6. 새 세트를 만들 때

- 생성 명령, 시드, 맵 수를 이 문서 2장 표에 한 줄 추가한다.
- 맵 파일은 `.gitignore`에 넣고 `index.csv`, `list.txt`, 미리보기만 올린다.
- 같은 생성기 안에서 시드 범위가 기존 세트와 겹치지 않게 한다. `make_suite.py`(gen_world): 0–39 평가, 1000– general200, 3000– obstacles100, 5000– official_like50, 7000– stress50, 9000– hills60. `gen_tiles.py`: 0 tiles60, 101 decoy_sample, 1000·1250·1500·1750 decoy1000.
