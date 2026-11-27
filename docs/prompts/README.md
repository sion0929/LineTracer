# 파트별 AI 시작 프롬프트

각 파일의 `---` 사이를 그 파트 AI에게 첫 메시지로 붙여 넣는다. 저장소 루트에서 실행한다.

| 파트 | 파일 | 맡는 노드 |
|---|---|---|
| 인지 | [`perception.md`](perception.md) | `perception_node`, `map_node` |
| 판단 | [`planning.md`](planning.md) | `planning_node` (지금 더미) |
| 제어 | [`control.md`](control.md) | `control_node` (지금 더미) |

공통 규칙은 `AGENTS.md`, 노드·토픽은 `docs/NODE_DEV_GUIDE.md`에 있다. 프롬프트는 그 문서들을 먼저 읽게 한다.
