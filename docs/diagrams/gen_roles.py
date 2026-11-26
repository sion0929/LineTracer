"""Generates roles.html: perception / planning / control responsibilities (diagram-design skin)."""
from pathlib import Path

INK, MUTED, SOFT, PAPER, ACC = "#2d3142", "#4f5d75", "#5a6580", "#f5f5f5", "#bf4520"
KR = "'Geist','Noto Sans KR','Apple SD Gothic Neo',sans-serif"
MONO = "'Geist Mono',ui-monospace,monospace"

cols = [
    dict(tag="PERCEPTION", name="인지", focal=True, status="우리 팀 · 개선 중",
         summary="선이 어디 있는지 정확히 기록한다",
         tasks=[("선 감지", "영상에서 선 픽셀 찾기 (임계값 · 분할 모델)"),
                ("BEV 투영", "바닥 좌표로 펴기, 경사로는 IMU · 깊이로 보정"),
                ("선 지도", "odom 좌표 5 mm 격자에 누적"),
                ("위치 보정", "엔코더 + IMU로 미끄러짐 보정"),
                ("선 복원", "안 보이는 구간을 학습 모델로 잇기")],
         out=("차선 경로 (Route)", "points · end: open | t")),
    dict(tag="PLANNING", name="판단", focal=False, status="기준 구현 고정",
         summary="어느 길로, 얼마나 빠르게 갈지 정한다",
         tasks=[("길 선택", "교차로 직진, 끊긴 길 잇기 (trace)"),
                ("종점 판정", "종점 T자 확인, 정지 위치 결정"),
                ("복구", "선을 놓쳤을 때 다시 찾기"),
                ("속도 계획", "곡률 · 경사 · 시야 끝에 맞춰 감속"),
                ("레이싱 라인", "허용 폭 안에서 더 빠른 경로")],
         out=("목표 궤적 (Trajectory)", "points · v_target · action")),
    dict(tag="CONTROL", name="제어", focal=False, status="기준 구현 고정",
         summary="궤적을 실제 바퀴 명령으로 바꾼다",
         tasks=[("경로 추종", "pure pursuit (Stanley · PID · MPC 후보)"),
                ("명령 제한", "가속도 · 각속도 한계 지키기"),
                ("제자리 회전", "급코너에서 회전 시작 각도 조절"),
                ("과속 억제", "내리막 · 미끄러짐에서 속도 줄이기"),
                ("실물 튜닝", "모터 PID, 오도메트리 보정 (Jetson)")],
         out=("속도 명령 (/cmd_vel)", "v [m/s] · ω [rad/s]")),
]

X0, W, GAP, Y0, H = 48, 352, 64, 40, 468
s = []
a = s.append
# arrows first
for i in range(2):
    x1 = X0 + i * (W + GAP) + W
    a(f'<line x1="{x1}" y1="{Y0+404}" x2="{x1+GAP-2}" y2="{Y0+404}" stroke="{ACC if i == 0 else MUTED}" stroke-width="1.6" marker-end="url(#{"arrow-accent" if i == 0 else "arrow"})"/>')
# robot -> perception, control -> robot
xa, xb = X0 + 176, X0 + 2 * (W + GAP) + 176
a(f'<line x1="{xa}" y1="580" x2="{xa}" y2="{Y0+H+2}" stroke="{MUTED}" stroke-width="1.4" marker-end="url(#arrow)"/>')
a(f'<line x1="{xb}" y1="{Y0+H}" x2="{xb}" y2="578" stroke="{MUTED}" stroke-width="1.4" marker-end="url(#arrow)"/>')
for x, lab in ((xa, "camera · IMU · odom"), (xb, "/cmd_vel")):
    w = len(lab) * 6.8 + 12
    a(f'<rect x="{x+10}" y="536" width="{w:.0f}" height="20" rx="2" fill="{PAPER}"/>')
    a(f'<text x="{x+16}" y="550" font-family="{MONO}" font-size="11" fill="{MUTED}">{lab}</text>')

for i, c in enumerate(cols):
    x = X0 + i * (W + GAP)
    st = ACC if c["focal"] else "#bfc0c0"
    fill = "rgba(191,69,32,0.06)" if c["focal"] else "#ffffff"
    a(f'<rect x="{x}" y="{Y0}" width="{W}" height="{H}" rx="8" fill="{PAPER}"/>')
    a(f'<rect x="{x}" y="{Y0}" width="{W}" height="{H}" rx="8" fill="{fill}" stroke="{st}" stroke-width="{1.6 if c["focal"] else 1}"/>')
    tw = len(c["tag"]) * 7 + 16
    a(f'<rect x="{x+20}" y="{Y0+20}" width="{tw}" height="18" rx="2" fill="none" stroke="{st}" stroke-width="0.8"/>')
    a(f'<text x="{x+28}" y="{Y0+33}" font-family="{MONO}" font-size="10" letter-spacing="0.08em" fill="{ACC if c["focal"] else SOFT}">{c["tag"]}</text>')
    a(f'<text x="{x+20}" y="{Y0+70}" font-family="{KR}" font-size="26" font-weight="600" fill="{INK}">{c["name"]}</text>')
    sw = sum(12 if ord(ch) > 0x1100 else 7 for ch in c["status"]) + 20
    sw = (sw + 3) // 4 * 4
    a(f'<rect x="{x+W-20-sw}" y="{Y0+52}" width="{sw}" height="24" rx="4" fill="{ACC if c["focal"] else "#ececec"}"/>')
    a(f'<text x="{x+W-20-sw/2}" y="{Y0+68}" text-anchor="middle" font-family="{KR}" font-size="12" font-weight="500" fill="{"#ffffff" if c["focal"] else MUTED}">{c["status"]}</text>')
    a(f'<text x="{x+20}" y="{Y0+98}" font-family="{KR}" font-size="14" fill="{MUTED}">{c["summary"]}</text>')
    a(f'<line x1="{x+20}" y1="{Y0+116}" x2="{x+W-20}" y2="{Y0+116}" stroke="rgba(45,49,66,0.12)"/>')
    for k, (n, d) in enumerate(c["tasks"]):
        y = Y0 + 144 + k * 44
        a(f'<text x="{x+20}" y="{y}" font-family="{KR}" font-size="14" font-weight="600" fill="{INK}">{n}</text>')
        a(f'<text x="{x+20}" y="{y+19}" font-family="{KR}" font-size="12.5" fill="{SOFT}">{d}</text>')
    oy = Y0 + 372
    a(f'<rect x="{x+16}" y="{oy}" width="{W-32}" height="80" rx="6" fill="{PAPER}" stroke="{st}" stroke-width="1" stroke-dasharray="4,3"/>')
    a(f'<text x="{x+32}" y="{oy+24}" font-family="{KR}" font-size="12" font-weight="500" fill="{SOFT}">출력</text>')
    a(f'<text x="{x+32}" y="{oy+46}" font-family="{KR}" font-size="15" font-weight="600" fill="{INK}">{c["out"][0]}</text>')
    a(f'<text x="{x+32}" y="{oy+66}" font-family="{MONO}" font-size="11" fill="{MUTED}">{c["out"][1]}</text>')

# robot
a(f'<rect x="{X0}" y="580" width="1184" height="76" rx="8" fill="#ececec" stroke="#bfc0c0"/>')
a(f'<text x="{X0+20}" y="612" font-family="{KR}" font-size="18" font-weight="600" fill="{INK}">로봇</text>')
a(f'<text x="{X0+20}" y="636" font-family="{KR}" font-size="13" fill="{MUTED}">TurtleBot3 Waffle Pi · Gazebo 시뮬레이션 / 실물 Jetson Orin Nano 8GB</text>')
a(f'<text x="{X0+1164}" y="626" text-anchor="end" font-family="{MONO}" font-size="11" fill="{SOFT}">ROS 2 Jazzy · follower_node</text>')
# legend strip
a(f'<rect x="{X0}" y="684" width="14" height="14" rx="3" fill="rgba(191,69,32,0.06)" stroke="{ACC}" stroke-width="1.4"/>')
a(f'<text x="{X0+22}" y="696" font-family="{KR}" font-size="12" fill="{MUTED}">우리 팀이 바꾸는 부분</text>')
a(f'<rect x="{X0+180}" y="684" width="14" height="14" rx="3" fill="#ffffff" stroke="#bfc0c0"/>')
a(f'<text x="{X0+202}" y="696" font-family="{KR}" font-size="12" fill="{MUTED}">고정 (기준 구현, 공식 트랙 약 64초)</text>')
a(f'<rect x="{X0+440}" y="684" width="28" height="14" rx="3" fill="{PAPER}" stroke="#bfc0c0" stroke-dasharray="4,3"/>')
a(f'<text x="{X0+476}" y="696" font-family="{KR}" font-size="12" fill="{MUTED}">다음 파트로 넘기는 인터페이스</text>')

svg = "\n".join(s)
tpl = Path(__file__).with_name("roles_template.html").read_text()
Path(__file__).with_name("roles.html").write_text(tpl.replace("<!--BODY-->", svg))
