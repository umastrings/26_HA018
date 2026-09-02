#!/usr/bin/env python3
"""
gen_hospital_4bed.py
  Smart Up-Care  4인실 + 복도 + 간호 스테이션 Gazebo Classic 11 월드 생성기 + 기하 검증기

  - 모든 구조물은 SDF primitive box (mesh/텍스처/온라인 모델 DB 미사용)
  - visual == collision (동일 size)
  - 모든 정적 구조물 <static>true</static>
  - pose z = height/2  (바닥 z=0 기준, 떠 있거나 묻히지 않음)

  검증:
    1) 정적 박스 상호 겹침 (침상-벽, 가구-침상 등)
    2) 로봇 스폰 자세 / Bed1~4 목표 자세에서 footprint AABB 겹침 + 최소 여유거리
    3) 여유거리를 Nav2 inscribed_radius / inflation_radius 와 비교

  실행: python3 gen_hospital_4bed.py
"""
import math
import xml.etree.ElementTree as ET

# ============================================================
# 0. 로봇 제원 — nav2_care.yaml 과 반드시 동일해야 함
#    footprint: [[0.2495,0.2255],[0.2495,-0.2255],[-0.2495,-0.2255],[-0.2495,0.2255]]
# ============================================================
FP_HALF_X = 0.2495          # 전후 반길이 (m)
FP_HALF_Y = 0.2255          # 좌우 반폭 (m)
R_INSCRIBED = FP_HALF_Y     # 내접원 = 0.2255 (이보다 가까우면 치명 비용)
R_INFLATION = 0.34          # nav2_care.yaml inflation_radius
LIDAR_Z = 0.175             # 스캔 평면 높이 → 이보다 낮은 물체는 LiDAR에 안 보임

WALL_T = 0.10               # 벽 두께
WALL_H = 1.00               # 벽 높이 (영상 촬영용 부감뷰 확보를 위해 낮게 유지)

# ============================================================
# 1. 치수 / 기준 좌표
#    X = 진행 방향(간호스테이션 → 복도 → 병실),  Y = 좌우(+Y = 북)
# ============================================================
# 간호 스테이션 내부: x[-2.0, 0.0], y[-1.0, 1.0]   (2.0 x 2.0)
NS_X0, NS_X1 = -2.00, 0.00
NS_Y0, NS_Y1 = -1.00, 1.00

# 복도 내부: x[0.0, 8.0], y[-0.6, 0.6]             (8.0 x 1.2)
CR_X0, CR_X1 = 0.00, 8.00
CR_Y0, CR_Y1 = -0.60, 0.60

# 병실 내부: x[8.0, 14.8], y[-2.5, 2.5]            (6.8 x 5.0)
RM_X0, RM_X1 = 8.00, 14.80
RM_Y0, RM_Y1 = -2.50, 2.50

DOOR_HALF = 0.70            # 병실 출입구 반폭 → 개구부 1.40 m (요구 1.2 m 이상 충족)
FLARE_X = 7.40              # 문 앞 확장(flare) 시작 X. 복도 1.2 m → 1.4 m 로 벌어짐
                            #   이게 없으면 복도 벽 끝단이 개구부를 1.20 m 로 잠식함

# 복도 벽면 리세스(SLAM 특징점 확보용, 복도를 좁히지 않고 넓히는 방향)
ALC_N_X0, ALC_N_X1 = 3.60, 4.40     # 북측 알코브 (린넨실) 깊이 0.6
ALC_N_Y1 = 1.20
ALC_S_X0, ALC_S_X1 = 5.60, 6.40     # 남측 리세스 (문틀) 깊이 0.6
ALC_S_Y1 = -1.20

# 침상 (2.0 x 0.9 x 0.5) — 북측 2개 / 남측 2개, 장축은 X
BED_L, BED_W, BED_H = 2.00, 0.90, 0.50
BED_X_W, BED_X_E = 10.20, 12.80             # 서측/동측 침상 X 중심
BED_Y_N = RM_Y1 - BED_W / 2                 # 2.05  (북벽 밀착)
BED_Y_S = RM_Y0 + BED_W / 2                 # -2.05 (남벽 밀착)

# 침상 옆 목표점: 침상 안쪽 면에서 0.35 m 떨어진 위치 (inflation 0.34 초과 확보)
BEDSIDE_Y_N = BED_Y_N - BED_W / 2 - 0.35 - FP_HALF_X   # yaw=+90deg → Y방향 반extent = FP_HALF_X
BEDSIDE_Y_S = -BEDSIDE_Y_N

# ============================================================
# 2. 박스 정의   (name, cx, cy, sx, sy, sz)   cz = sz/2 자동
# ============================================================
boxes = []


def box(name, cx, cy, sx, sy, sz, color="wall"):
    boxes.append(dict(name=name, cx=cx, cy=cy, cz=sz / 2.0,
                      sx=sx, sy=sy, sz=sz, color=color))


def wall_x(name, x0, x1, y_face, side, h=WALL_H):
    """X방향으로 뻗은 벽. y_face = 실내쪽 벽면, side=+1이면 벽이 +Y쪽으로 두께를 가짐"""
    cy = y_face + side * WALL_T / 2.0
    box(name, (x0 + x1) / 2.0, cy, x1 - x0, WALL_T, h)


def wall_y(name, y0, y1, x_face, side, h=WALL_H):
    """Y방향으로 뻗은 벽. x_face = 실내쪽 벽면"""
    cx = x_face + side * WALL_T / 2.0
    box(name, cx, (y0 + y1) / 2.0, WALL_T, y1 - y0, h)


# ---------- 간호 스테이션 ----------
wall_y("ns_wall_west",   NS_Y0 - WALL_T, NS_Y1 + WALL_T, NS_X0, -1)
wall_x("ns_wall_north",  NS_X0 - WALL_T, NS_X1,          NS_Y1, +1)
wall_x("ns_wall_south",  NS_X0 - WALL_T, NS_X1,          NS_Y0, -1)
# 스테이션(폭 2.0) → 복도(폭 1.2) 목 좁힘. 복도 입구 y[-0.6, 0.6]만 열림
box("ns_neck_north", NS_X1 - WALL_T / 2, (CR_Y1 + NS_Y1 + WALL_T) / 2,
    WALL_T, (NS_Y1 + WALL_T) - CR_Y1, WALL_H)
box("ns_neck_south", NS_X1 - WALL_T / 2, (CR_Y0 + NS_Y0 - WALL_T) / 2,
    WALL_T, CR_Y0 - (NS_Y0 - WALL_T), WALL_H)

# ---------- 복도 (알코브로 분절) ----------
wall_x("cr_wall_n_1", CR_X0, ALC_N_X0, CR_Y1, +1)
wall_x("cr_wall_n_2", ALC_N_X1, FLARE_X, CR_Y1, +1)
wall_x("alcove_n_back", ALC_N_X0 - WALL_T, ALC_N_X1 + WALL_T, ALC_N_Y1, +1)
wall_y("alcove_n_side_w", CR_Y1, ALC_N_Y1 + WALL_T, ALC_N_X0, -1)
wall_y("alcove_n_side_e", CR_Y1, ALC_N_Y1 + WALL_T, ALC_N_X1, +1)
# 문 앞 확장: 1.2 m -> 1.4 m 단차 + 확장 구간 벽
wall_y("cr_step_n", CR_Y1, DOOR_HALF + WALL_T, FLARE_X, -1)
wall_x("cr_wall_n_3", FLARE_X - WALL_T, RM_X0, DOOR_HALF, +1)

wall_x("cr_wall_s_1", CR_X0, ALC_S_X0, CR_Y0, -1)
wall_x("cr_wall_s_2", ALC_S_X1, FLARE_X, CR_Y0, -1)
wall_x("alcove_s_back", ALC_S_X0 - WALL_T, ALC_S_X1 + WALL_T, ALC_S_Y1, -1)
wall_y("alcove_s_side_w", ALC_S_Y1 - WALL_T, CR_Y0, ALC_S_X0, -1)
wall_y("alcove_s_side_e", ALC_S_Y1 - WALL_T, CR_Y0, ALC_S_X1, +1)
wall_y("cr_step_s", -(DOOR_HALF + WALL_T), CR_Y0, FLARE_X, -1)
wall_x("cr_wall_s_3", FLARE_X - WALL_T, RM_X0, -DOOR_HALF, -1)

# ---------- 병실 ----------
wall_y("rm_wall_w_n", DOOR_HALF, RM_Y1 + WALL_T, RM_X0, -1)
wall_y("rm_wall_w_s", RM_Y0 - WALL_T, -DOOR_HALF, RM_X0, -1)
wall_y("rm_wall_east", RM_Y0 - WALL_T, RM_Y1 + WALL_T, RM_X1, +1)
wall_x("rm_wall_north", RM_X0 - WALL_T, RM_X1 + WALL_T, RM_Y1, +1)
wall_x("rm_wall_south", RM_X0 - WALL_T, RM_X1 + WALL_T, RM_Y0, -1)

# ---------- 침상 4개 ----------
box("bed_1", BED_X_W, BED_Y_N, BED_L, BED_W, BED_H, "bed")   # 북서
box("bed_2", BED_X_E, BED_Y_N, BED_L, BED_W, BED_H, "bed")   # 북동
box("bed_3", BED_X_W, BED_Y_S, BED_L, BED_W, BED_H, "bed")   # 남서
box("bed_4", BED_X_E, BED_Y_S, BED_L, BED_W, BED_H, "bed")   # 남동

# ---------- 병실 가구 (LiDAR 특징점용, 중앙 통로 |y|<=1.6 침범 금지) ----------
# 침상 사이 커튼 파티션 (침상 간격 x[11.20,11.80] 안쪽)
box("curtain_n", 11.50, BED_Y_N, 0.08, BED_W, WALL_H, "curtain")
box("curtain_s", 11.50, BED_Y_S, 0.08, BED_W, WALL_H, "curtain")
# 협탁 (출입구 쪽, 침상 서단에 인접)
box("nightstand_1", 8.95, 2.25, 0.40, 0.40, 0.60, "furn")
box("nightstand_2", 8.95, -2.25, 0.40, 0.40, 0.60, "furn")
# 동측 코너 세면대 / 캐비닛
box("sink",       14.45, 2.28, 0.50, 0.40, 0.85, "furn")
box("cabinet_se", 14.45, -2.28, 0.50, 0.40, 0.90, "furn")
# 동벽 중앙 낮은 캐비닛 (동벽 특징점)
box("cabinet_e",  14.50, 0.00, 0.40, 1.20, 0.70, "furn")

# ---------- 간호 스테이션 가구 ----------
box("ns_desk",    -1.65, 0.35, 0.60, 1.30, 0.75, "furn")
box("ns_chair",   -1.12, 0.80, 0.40, 0.40, 0.50, "furn")   # 책상과 0.03 m 이격
box("ns_cabinet", -1.75, -0.65, 0.40, 0.60, 0.85, "furn")

# ============================================================
# 3. 로봇 자세 정의
# ============================================================
SPAWN = ("robot_spawn", -0.60, 0.00, 0.0)
POSES = [
    SPAWN,
    ("goal_nurse_station", -0.70, 0.00, math.pi),
    ("goal_corridor_entry", 1.00, 0.00, 0.0),
    ("goal_room_entry", 8.60, 0.00, 0.0),
    ("goal_bed_1", BED_X_W, BEDSIDE_Y_N, math.pi / 2),
    ("goal_bed_2", BED_X_E, BEDSIDE_Y_N, math.pi / 2),
    ("goal_bed_3", BED_X_W, BEDSIDE_Y_S, -math.pi / 2),
    ("goal_bed_4", BED_X_E, BEDSIDE_Y_S, -math.pi / 2),
]

OBSTACLES = [
    ("복도-알코브구간(회피)", 4.00, -0.30),
    ("복도-직선부(정지)", 2.00, -0.30),
    ("병실 중앙통로(회피)", 11.50, 0.00),
    ("병실 출입구 앞(회피)", 8.90, 0.35),
]

# ============================================================
# 4. 검증
# ============================================================
def aabb(b):
    return (b["cx"] - b["sx"] / 2, b["cx"] + b["sx"] / 2,
            b["cy"] - b["sy"] / 2, b["cy"] + b["sy"] / 2)


def fp_aabb(x, y, yaw):
    """직사각 footprint를 yaw 회전시킨 뒤의 축정렬 반extent"""
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    hx = FP_HALF_X * c + FP_HALF_Y * s
    hy = FP_HALF_X * s + FP_HALF_Y * c
    return (x - hx, x + hx, y - hy, y + hy)


def gap(a, b):
    """두 AABB 사이 거리. 겹치면 음수(겹침 깊이)"""
    dx = max(a[0] - b[1], b[0] - a[1])
    dy = max(a[2] - b[3], b[2] - a[3])
    if dx <= 0 and dy <= 0:
        return -min(-dx, -dy)
    if dx > 0 and dy > 0:
        return math.hypot(dx, dy)
    return max(dx, dy)


report = []
report.append("=" * 78)
report.append(" [1] 정적 박스 상호 겹침 검사")
report.append("=" * 78)
overlaps, corner_joins = 0, 0
for i in range(len(boxes)):
    for j in range(i + 1, len(boxes)):
        bi, bj = boxes[i], boxes[j]
        g = gap(aabb(bi), aabb(bj))
        if g >= -1e-9:
            continue
        # 벽-벽 코너 접합은 의도된 겹침 (침투 <= 벽두께)
        if (bi["color"] == "wall" and bj["color"] == "wall"
                and -g <= WALL_T + 1e-6):
            corner_joins += 1
            continue
        overlaps += 1
        report.append(f"  [겹침] {bi['name']:18s} <-> {bj['name']:18s} "
                      f"침투 {-g:.4f} m")
report.append(f"  박스 {len(boxes)}개 / 벽 코너 접합(의도됨) {corner_joins}건 "
              f"/ 비정상 겹침 {overlaps}건"
              + ("  -> PASS" if overlaps == 0 else "  -> FAIL"))

report.append("")
report.append("=" * 78)
report.append(" [2] 로봇 자세별 여유거리 (footprint AABB 기준)")
report.append(f"     내접원 {R_INSCRIBED:.4f} m / inflation_radius {R_INFLATION:.2f} m")
report.append("=" * 78)
pose_fail = 0
for name, x, y, yaw in POSES:
    fa = fp_aabb(x, y, yaw)
    worst, worst_name = 1e9, "-"
    for b in boxes:
        g = gap(fa, aabb(b))
        if g < worst:
            worst, worst_name = g, b["name"]
    if worst < 0:
        verdict, bad = "FAIL 내부충돌", True
    elif worst < R_INSCRIBED:
        verdict, bad = "FAIL 치명비용", True
    elif worst < R_INFLATION:
        verdict, bad = "WARN 인플레이션내", True
    else:
        verdict, bad = "PASS", False
    pose_fail += bad
    report.append(f"  {name:22s} ({x:6.2f},{y:6.2f}, yaw {math.degrees(yaw):6.1f}deg) "
                  f"최근접 {worst_name:16s} {worst:6.3f} m  {verdict}")
report.append(f"  결과: {len(POSES)}개 자세 중 문제 {pose_fail}건"
              + ("  -> PASS" if pose_fail == 0 else "  -> 확인필요"))

report.append("")
report.append("=" * 78)
report.append(" [3] 실제 기하로 스캔한 통행 유효폭 (y=0 을 포함하는 자유 구간)")
report.append("=" * 78)


def free_width_at(x, extra=()):
    """주어진 x 단면에서 y=0 을 포함하는 자유 구간 [y_lo, y_hi] 를 실제 박스로부터 산출"""
    blocked = []
    for b in list(boxes) + list(extra):
        a = aabb(b)
        if a[0] - 1e-9 <= x <= a[1] + 1e-9:
            blocked.append((a[2], a[3]))
    lo, hi = -50.0, 50.0
    for y0, y1 in blocked:
        if y1 <= 0:
            lo = max(lo, y1)
        elif y0 >= 0:
            hi = min(hi, y0)
        else:
            return (0.0, 0.0)      # y=0 자체가 막힘
    return (lo, hi)


scans = [
    ("간호스테이션 중앙", -1.00), ("복도 입구", 0.30),
    ("복도 직선부", 2.00), ("알코브 구간", 4.00),
    ("남측 리세스 구간", 6.00), ("문 앞 확장 구간", 7.70),
    ("병실 개구부(벽면)", 7.95), ("병실 진입 직후", 8.60),
    ("Bed1/Bed3 사이", 10.20), ("커튼 구간", 11.50),
    ("Bed2/Bed4 사이", 12.80), ("병실 동단", 14.20),
]
width_fail = 0
for label, x in scans:
    lo, hi = free_width_at(x)
    w = hi - lo
    slack = w - 2 * FP_HALF_Y
    if w <= 0:
        v, bad = "FAIL 막힘", True
    elif slack / 2 < R_INSCRIBED:
        v, bad = "FAIL 통과불가", True
    elif slack / 2 < R_INFLATION:
        v, bad = "WARN 인플레이션내", True
    else:
        v, bad = "PASS", False
    width_fail += bad
    report.append(f"  x={x:6.2f} {label:20s} 자유폭 {w:5.2f} m "
                  f"(y {lo:+.2f}~{hi:+.2f}) 편측여유 {slack/2:5.3f} m  {v}")
report.append(f"  결과: {len(scans)}개 단면 중 문제 {width_fail}건"
              + ("  -> PASS" if width_fail == 0 else "  -> 확인필요"))
report.append(f"  침상 간 간격(커튼 두께 0.08 포함) "
              f"{(BED_X_E - BED_L/2) - (BED_X_W + BED_L/2):.2f} m")

report.append("")
report.append("=" * 78)
report.append(" [4] LiDAR 가시성 (스캔 평면 z=%.3f m)" % LIDAR_Z)
report.append("=" * 78)
invisible = [b["name"] for b in boxes if b["sz"] <= LIDAR_Z]
report.append(f"  스캔 평면보다 낮아 지도에 안 잡히는 물체: "
              f"{invisible if invisible else '없음 (전부 가시)'}")

report.append("")
report.append("=" * 78)
report.append(" [5] 데모 장애물 배치 시 잔여 통행폭")
report.append("=" * 78)
OB = 0.40   # demo_box / wheelchair 외곽 한 변


def widest_free_side(x, ob):
    """장애물을 놓은 뒤 그 단면에서 확보되는 가장 넓은 자유 구간 폭"""
    blocked = []
    for b in list(boxes) + [ob]:
        a = aabb(b)
        if a[0] - 1e-9 <= x <= a[1] + 1e-9:
            blocked.append((a[2], a[3]))
    blocked.sort()
    best, cur = 0.0, -50.0
    merged = []
    for y0, y1 in blocked:
        if merged and y0 <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], y1))
        else:
            merged.append((y0, y1))
    for y0, y1 in merged:
        best = max(best, y0 - cur if cur > -49 else 0.0)
        cur = y1
    return best


for label, ox, oy in OBSTACLES:
    ob = dict(name="obstacle", cx=ox, cy=oy, cz=OB / 2,
              sx=OB, sy=OB, sz=0.60, color="furn")
    w = widest_free_side(ox, ob)
    slack = (w - 2 * FP_HALF_Y) / 2
    if slack >= R_INFLATION:
        v = "회피 가능 (여유 충분)"
    elif slack >= R_INSCRIBED:
        v = "통과 가능하나 인플레이션 비용 발생"
    else:
        v = "통과 불가 -> 정지/대기 데모용"
    report.append(f"  {label:18s} ({ox:5.2f},{oy:5.2f})  최광 자유폭 {w:5.2f} m  "
                  f"편측여유 {slack:5.3f} m  -> {v}")

VERIFY = "\n".join(report)
print(VERIFY)

# ============================================================
# 5. SDF 생성
# ============================================================
COLORS = {
    "wall":    ("0.60 0.62 0.64 1", "0.82 0.84 0.86 1"),
    "bed":     ("0.30 0.42 0.58 1", "0.55 0.70 0.86 1"),
    "curtain": ("0.35 0.50 0.55 1", "0.62 0.80 0.84 1"),
    "furn":    ("0.45 0.38 0.30 1", "0.72 0.63 0.52 1"),
}


def model_xml(b):
    amb, dif = COLORS[b["color"]]
    return f"""
    <model name="{b['name']}">
      <static>true</static>
      <pose>{b['cx']:.4f} {b['cy']:.4f} {b['cz']:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{b['sx']:.4f} {b['sy']:.4f} {b['sz']:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{b['sx']:.4f} {b['sy']:.4f} {b['sz']:.4f}</size></box></geometry>
          <material>
            <ambient>{amb}</ambient>
            <diffuse>{dif}</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


header = f"""<?xml version="1.0" ?>
<!--
  hospital_4bed.world  —  Smart Up-Care 시연용 4인실 병동 (Gazebo Classic 11)
  자동 생성: gen_hospital_4bed.py  (직접 편집하지 말고 생성기를 고칠 것)

  좌표계: X = 진행방향(간호스테이션 -> 복도 -> 병실), Y = 좌우(+Y = 북)
  구성:  간호스테이션 2.0x2.0  |  복도 8.0x1.2  |  4인실 6.8x5.0
  모든 구조물 SDF primitive box / visual == collision / static=true
  mesh, 텍스처, 온라인 모델 DB 미사용 (ground_plane, sun 은 Gazebo 로컬 기본 모델)

  ★ VM 성능 옵션: <scene> shadows=false, <physics> max_step_size 0.002.
    기존 care_facility.world 와 물리 스텝이 달라 EKF/오도메트리 거동이 다르면
    아래 <physics> 블록을 지워 Gazebo 기본값(0.001/1000Hz)으로 되돌릴 것.
-->
<sdf version="1.6">
  <world name="hospital_4bed">

    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>

    <!-- VM(llvmpipe) 렌더 부하 절감: 그림자 off -->
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.85 0.88 0.92 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

    <physics type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>

    <!-- 영상 촬영용 기본 카메라: 복도 서측에서 병실 방향 부감 -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-2.5 -6.0 6.5 0 0.62 0.95</pose>
      </camera>
    </gui>
"""

sections = [
    ("간호 스테이션 (2.0 x 2.0) — 로봇 시작/복귀 지점", "ns_"),
    ("복도 (8.0 x 1.2) + 벽면 알코브(SLAM 특징점) + 문앞 확장", ("cr_", "alcove_")),
    ("병실 벽 (6.8 x 5.0), 출입구 폭 1.40", "rm_"),
    ("침상 4개 (2.0 x 0.9 x 0.5)", "bed_"),
    ("병실 가구 / 커튼 파티션", ("curtain_", "nightstand_", "sink", "cabinet_")),
]
emitted, body = set(), []
for title, pref in sections:
    prefs = pref if isinstance(pref, tuple) else (pref,)
    grp = [b for b in boxes if b["name"].startswith(prefs)
           and b["name"] not in emitted]
    if not grp:
        continue
    body.append(f"\n    <!-- ==================== {title} ==================== -->")
    for b in grp:
        emitted.add(b["name"])
        body.append(model_xml(b))
rest = [b for b in boxes if b["name"] not in emitted]
if rest:
    body.append("\n    <!-- ==================== 기타 ==================== -->")
    for b in rest:
        body.append(model_xml(b))

world = header + "".join(body) + "\n\n  </world>\n</sdf>\n"

OUT = "hospital_4bed.world"
with open(OUT, "w", encoding="utf-8") as f:
    f.write(world)

ET.fromstring(world)   # XML 문법 검증 (실패하면 예외)
print(f"\n[OK] {OUT} 생성 — 모델 {len(boxes)}개, XML 파싱 통과")

# 좌표표 CSV
with open("hospital_4bed_coords.csv", "w", encoding="utf-8") as f:
    f.write("name,pose_x,pose_y,pose_z,size_x,size_y,size_z,x_min,x_max,y_min,y_max\n")
    for b in sorted(boxes, key=lambda d: d["name"]):
        a = aabb(b)
        f.write(f"{b['name']},{b['cx']:.4f},{b['cy']:.4f},{b['cz']:.4f},"
                f"{b['sx']:.4f},{b['sy']:.4f},{b['sz']:.4f},"
                f"{a[0]:.4f},{a[1]:.4f},{a[2]:.4f},{a[3]:.4f}\n")
with open("verify_report.txt", "w", encoding="utf-8") as f:
    f.write(VERIFY + "\n")
print("[OK] hospital_4bed_coords.csv / verify_report.txt 생성")
print(f"\n침상 옆 목표 Y: 북측 {BEDSIDE_Y_N:+.4f} / 남측 {BEDSIDE_Y_S:+.4f}")
