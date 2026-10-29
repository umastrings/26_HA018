# hospital_4bed — 4인실 시연 월드 (Gazebo Classic 11 / ROS2 Humble)

> Smart Up-Care 한이음 시연 영상용 가상월드.
> 기존 `care_facility.world` / `spawn.launch.py` / `sim_bringup.launch.py` 는 **전혀 변경하지 않음**.

---

## 1. 추가된 파일

| 파일 | 역할 |
|---|---|
| `src/smart_upcare/worlds/hospital_4bed.world` | 신규 월드 (모델 38개, 전부 primitive box) |
| `src/smart_upcare/worlds/gen_hospital_4bed.py` | 월드 생성기 + 기하 검증기 (수정은 이 파일에서) |
| `src/smart_upcare/worlds/hospital_4bed_coords.csv` | 전체 좌표·크기·AABB 표 |
| `src/smart_upcare/worlds/models/demo_box.sdf` | 데모 장애물 (박스 0.4×0.4×0.6) |
| `src/smart_upcare/worlds/models/demo_wheelchair.sdf` | 데모 장애물 (휠체어, box+cylinder) |
| `src/smart_upcare_control/launch/gazebo_4bed.launch.py` | 4인실 월드 원커맨드 브링업 |
| `src/smart_upcare_control/launch/spawn_obstacle.launch.py` | 실행 중 장애물 투입 |

`smart_upcare/CMakeLists.txt` 는 `install(DIRECTORY ... worlds ...)` 로 **재귀 복사**하므로
`worlds/models/` 하위도 자동 설치됨 → **빌드 설정 변경 불필요**.

---

## 2. 실행

```bash
cd ~/hanium_ws
colcon build --packages-select smart_upcare smart_upcare_control
source install/setup.bash

# 월드 + 로봇 + EKF + 리프트 TF (원커맨드)
ros2 launch smart_upcare_control gazebo_4bed.launch.py
#   VM FPS 낮으면:
ros2 launch smart_upcare_control gazebo_4bed.launch.py gui:=false
```

> Gazebo 는 항상 일시정지로 시작 → 헤드리스일 때는
> `ros2 service call /unpause_physics std_srvs/srv/Empty {}` 필요.

### SLAM → 지도 저장

```bash
# 터미널 2
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true base_frame:=base_footprint

# 터미널 3 (텔레옵으로 전 구역 한 바퀴 돌린 뒤)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_smoothed -p use_sim_time:=true

ros2 run nav2_map_server map_saver_cli -f $HOME/hospital_4bed_map \
    --ros-args -p use_sim_time:=true
```

### Nav2 자율주행

```bash
ros2 launch smart_upcare_control navigation.launch.py \
    use_sim_time:=true map:=$HOME/hospital_4bed_map.yaml
```

### 데모 시나리오 (기존 노드에 새 좌표 주입 — 코드 수정 없음)

```bash
ros2 run smart_upcare_control demo_scenario --ros-args \
  -p use_sim_time:=true \
  -p entry_xy:="[1.0, 0.0]" \
  -p bedside_xy:="[10.2, 1.0]" \
  -p exit_xy:="[-0.7, 0.0]" \
  -p lift_up_mm:=700.0 -p lift_down_mm:=100.0
```

### 장애물 투입 / 제거

```bash
# 병실 중앙 통로 — 회피 + 재계획 (권장, 가장 안정적)
ros2 launch smart_upcare_control spawn_obstacle.launch.py x:=11.5 y:=0.0

# 복도 알코브 구간 — 좁은 복도 우회 (x 는 반드시 3.6~4.4)
ros2 launch smart_upcare_control spawn_obstacle.launch.py \
    model:=demo_wheelchair name:=wc x:=4.0 y:=-0.30 yaw:=1.5708

# 복도 직선부 — 통과 불가 → Collision Monitor 정지/대기
ros2 launch smart_upcare_control spawn_obstacle.launch.py x:=2.0 y:=-0.30

# 제거
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'demo_obstacle'}"
```

---

## 3. 좌표계 및 구역

X = 진행 방향, Y = 좌우 (+Y = 북), 바닥 z = 0.

| 구역 | X 범위 | Y 범위 | 크기 |
|---|---|---|---|
| 간호 스테이션 | −2.00 ~ 0.00 | −1.00 ~ +1.00 | 2.0 × 2.0 |
| 복도 | 0.00 ~ 7.40 | −0.60 ~ +0.60 | 7.4 × 1.2 |
| 문 앞 확장 | 7.40 ~ 8.00 | −0.70 ~ +0.70 | 0.6 × 1.4 |
| 4인실 | 8.00 ~ 14.80 | −2.50 ~ +2.50 | 6.8 × 5.0 |
| 북측 알코브 | 3.60 ~ 4.40 | +0.60 ~ +1.20 | 0.8 × 0.6 |
| 남측 리세스 | 5.60 ~ 6.40 | −1.20 ~ −0.60 | 0.8 × 0.6 |

복도 총연장 = 스테이션 목 ~ 병실 개구부 = **8.00 m** (요구 충족).
알코브/리세스는 8 m 평행 복도의 X방향 위치 모호성(aperture problem)을 없애기 위한
SLAM 특징점이며, 복도를 좁히지 않고 **넓히는 방향**으로만 만들었다.

---

## 4. 침상 배치 및 Nav2 목표 좌표

침상은 **북측 2개 / 남측 2개**, 장축은 X 방향, 벽에 밀착.

| 침상 | 위치 | 중심 (x, y) | 크기 | 점유 X | 점유 Y |
|---|---|---|---|---|---|
| Bed 1 | 북서 | (10.20, +2.05) | 2.0 × 0.9 × 0.5 | 9.20 ~ 11.20 | +1.60 ~ +2.50 |
| Bed 2 | 북동 | (12.80, +2.05) | 2.0 × 0.9 × 0.5 | 11.80 ~ 13.80 | +1.60 ~ +2.50 |
| Bed 3 | 남서 | (10.20, −2.05) | 2.0 × 0.9 × 0.5 | 9.20 ~ 11.20 | −2.50 ~ −1.60 |
| Bed 4 | 남동 | (12.80, −2.05) | 2.0 × 0.9 × 0.5 | 11.80 ~ 13.80 | −2.50 ~ −1.60 |

### 권장 Nav2 Goal

| 목표 | x | y | yaw (rad) | yaw (deg) | 침상까지 여유 |
|---|---|---|---|---|---|
| 로봇 스폰 | −0.60 | 0.00 | 0.0000 | 0 | — (최근접 0.381 m) |
| 간호 스테이션 복귀 | −0.70 | 0.00 | 3.1416 | 180 | — (최근접 0.375 m) |
| 복도 진입 | 1.00 | 0.00 | 0.0000 | 0 | — (최근접 0.374 m) |
| 병실 진입 | 8.60 | 0.00 | 0.0000 | 0 | — (최근접 0.590 m) |
| **Bed 1 옆** | **10.20** | **+1.00** | **+1.5708** | **+90** | **0.350 m** |
| **Bed 2 옆** | **12.80** | **+1.00** | **+1.5708** | **+90** | **0.350 m** |
| **Bed 3 옆** | **10.20** | **−1.00** | **−1.5708** | **−90** | **0.350 m** |
| **Bed 4 옆** | **12.80** | **−1.00** | **−1.5708** | **−90** | **0.350 m** |

yaw ±90° = 침상을 정면으로 바라보는 자세. 리프트 상승 + 카메라 하향 틸트 20° 로
침상 상면을 보게 되는 구도.

여유 0.350 m 는 `nav2_care.yaml` 의 `inflation_radius: 0.34` 를 **의도적으로 살짝 넘긴 값**이다.
더 가까이 붙이면 목표점이 인플레이션 영역에 들어가 Nav2 가 goal 을 거부하거나 진동한다.

---

## 5. 가구 좌표 (LiDAR 특징점용)

| 이름 | 중심 (x, y) | 크기 (x×y×z) | 비고 |
|---|---|---|---|
| curtain_n / curtain_s | (11.50, ±2.05) | 0.08 × 0.90 × 1.00 | 침상 사이 커튼 파티션 |
| nightstand_1 / _2 | (8.95, ±2.25) | 0.40 × 0.40 × 0.60 | 협탁 |
| sink | (14.45, +2.28) | 0.50 × 0.40 × 0.85 | 세면대 (북동 코너) |
| cabinet_se | (14.45, −2.28) | 0.50 × 0.40 × 0.90 | 캐비닛 (남동 코너) |
| cabinet_e | (14.50, 0.00) | 0.40 × 1.20 × 0.70 | 동벽 중앙 낮은 캐비닛 |
| ns_desk | (−1.65, +0.35) | 0.60 × 1.30 × 0.75 | 간호 데스크 |
| ns_chair | (−1.12, +0.80) | 0.40 × 0.40 × 0.50 | 의자 |
| ns_cabinet | (−1.75, −0.65) | 0.40 × 0.60 × 0.85 | 스테이션 캐비닛 |

전체 38개 모델의 정확한 좌표·AABB 는 `hospital_4bed_coords.csv` 참고.
벽 두께 0.10 m, 벽 높이 1.00 m (영상 촬영용 부감뷰 확보를 위해 낮게 유지).

---

## 6. 검증 결과 (`gen_hospital_4bed.py` 자동 산출)

로봇 제원은 `nav2_care.yaml` 의 footprint 를 그대로 사용:
`±0.2495 (x) × ±0.2255 (y)`, 내접원 0.2255 m, `inflation_radius` 0.34 m.

| 검사 | 결과 |
|---|---|
| 정적 박스 상호 겹침 | 벽 코너 접합(의도) 22건, **비정상 겹침 0건 → PASS** |
| `visual` == `collision` 크기 | 38/38 일치 → **PASS** |
| `<static>true</static>` | 38/38 → **PASS** |
| `pose z` == 높이/2 | 38/38 → **PASS** (바닥에 뜨거나 묻힌 모델 없음) |
| 로봇 스폰·목표 8개 자세 여유 | 최소 0.350 m ≥ 0.34 → **문제 0건 PASS** |
| 통행 유효폭 12개 단면 스캔 | 최소 편측여유 0.374 m → **문제 0건 PASS** |
| LiDAR 가시성 (z=0.175 m) | 스캔 평면보다 낮은 구조물 **없음** |

### 통행 유효폭 스캔 (실제 박스 기하로 산출)

| 단면 | 자유폭 | 편측 여유 |
|---|---|---|
| x=0.30 복도 입구 | 1.20 m | 0.374 m |
| x=2.00 복도 직선부 | 1.20 m | 0.374 m |
| x=4.00 알코브 구간 | 1.80 m | 0.674 m |
| x=7.95 병실 개구부 | 1.40 m | 0.474 m |
| x=10.20 Bed1/Bed3 사이 | 3.20 m | 1.374 m |
| x=11.50 커튼 구간 | 3.20 m | 1.374 m |

### 장애물 배치별 통행 가능성

| 배치 | 최광 자유폭 | 편측 여유 | 판정 |
|---|---|---|---|
| 병실 중앙 (11.50, 0.00) | 1.40 m | 0.474 m | 회피 가능 (권장) |
| 병실 출입구 앞 (8.90, +0.35) | 2.20 m | 0.874 m | 회피 가능 |
| 복도 알코브 (4.00, −0.30) | 1.30 m | 0.424 m | 회피 가능 — **알코브 경유 필수** |
| 복도 직선부 (2.00, −0.30) | 0.70 m | 0.124 m | **통과 불가** → 정지/대기 데모용 |

> 복도(1.2 m)에 0.4 m 장애물을 놓으면 남은 폭이 0.7 m 로 로봇 폭 0.451 m + 인플레이션을
> 감당할 수 없다. 그래서 **복도 회피 데모는 알코브가 있는 x 3.6~4.4 구간에서만 성립**한다.
> 이건 버그가 아니라 의도된 설계이고, 반대로 "정지 데모"가 필요할 때는 직선부에 놓으면 된다.

---

## 7. 시나리오 성립 여부

| # | 시나리오 | 성립 | 근거 |
|---|---|---|---|
| 1 | 간호 스테이션에서 시작 | ○ | 스폰 (−0.60, 0.00, 0°), 최근접 여유 0.381 m |
| 2 | 폭 1.2 m 복도 자율주행 | ○ | 편측 여유 0.374 m > inflation 0.34 |
| 3 | 4인실 진입 | ○ | 개구부 1.40 m, 편측 여유 0.474 m |
| 4 | Bed 1~4 옆 목표 도착 | ○ | 4개 목표 전부 여유 0.350 m |
| 5 | 중앙 통로 장애물 회피/재계획 | ○ | (11.50, 0.00) 배치 시 양측 1.40 m 확보 |
| 6 | 침상 옆 정지 후 메카넘 횡이동 + 리프트/카메라 TF | ○ | 중앙 통로 3.2 m → 횡이동 여유 충분. 리프트 TF 는 기존 `lift_tf` + `virtual_lift` 그대로 |

---

## 8. 실제로 실행해서 확인하지 못한 항목 (반드시 직접 확인)

이 작업은 ROS2/Gazebo 가 없는 환경에서 수행했다. **아래는 전부 미검증**이다.

1. **Gazebo 실제 로드** — `gzserver` 가 `hospital_4bed.world` 를 파싱하는지.
   XML 문법과 SDF 1.6 태그 구조만 확인함. SDF 스키마 검증(`gz sdf -k`) 미실시.
2. **`model://ground_plane`, `model://sun` 해석** — 기존 `care_facility.world` 가
   동작한다는 사실에 근거해 로컬 모델 DB에 존재한다고 가정했다. 없으면 다운로드를 시도한다.
3. **`<physics> max_step_size 0.002`** — 기존 월드(Gazebo 기본 0.001/1000 Hz)와 다르다.
   EKF `/odometry/filtered` 주기나 공분산 거동이 기존과 달라지면
   `hospital_4bed.world` 의 `<physics>` 블록을 **삭제**해 기본값으로 되돌릴 것.
4. **`<gui><camera>` 초기 시점** — 눈으로 확인해야 함.
5. **VM FPS / RTF** — 모델 38개 전부 primitive box, 그림자 off 지만 실측 필요.
   판단 기준: **FPS ≥ 20 이면 진행**. RTF 저하는 후보정 배속으로 해결 가능.
6. **SLAM 지도 품질** — 알코브/리세스가 실제로 X방향 모호성을 해소하는지는 돌려봐야 안다.
   루프 클로저가 안 잡히면 복도에 특징 구조를 더 추가할 것.
7. **Nav2 실제 경로 생성** — 여유 계산은 AABB 기반 정적 검사다. MPPI 가 실제로
   0.374 m 편측 여유 복도를 통과하는지는 `critics` 가중치에 달려 있어 튜닝이 필요할 수 있다.
8. **스폰 롤오버** — 기존에 관찰된 스폰 시 롤 문제. `spawn_z` 기본 0.10 유지했으나
   재현되면 URDF 바퀴 collision 을 sphere 로 바꾸는 기존 가설을 시험할 것.
9. **`spawn_obstacle.launch.py` 의 `PathJoinSubstitution([model, '.sdf'])`** —
   문자열 결합 substitution 문법이 Humble 에서 의도대로 동작하는지 미검증.
   실패하면 `model` 인자를 파일명 전체(`demo_box.sdf`)로 받도록 바꾸면 된다.
10. **`demo_wheelchair` 의 costmap 반영** — 스캔 평면 z=0.175 m 에서는 대차륜/캐스터만
    검출된다. costmap 에 확실한 덩어리로 찍히길 원하면 `demo_box` 를 쓸 것.

---

## 9. 월드 수정 방법

`hospital_4bed.world` 를 직접 편집하지 말고 생성기를 고칠 것:

```bash
cd ~/hanium_ws/src/smart_upcare/worlds
# 치수/좌표 수정 후
python3 gen_hospital_4bed.py     # 검증 리포트 출력 + world/csv 재생성
```

생성기가 좌표 겹침·통행폭·목표점 여유를 전부 재검사하므로,
치수를 바꿔도 시나리오가 깨지지 않는지 즉시 확인된다.
