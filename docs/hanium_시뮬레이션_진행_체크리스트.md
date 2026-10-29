# 4인실 자율주행 시뮬레이션 — 단계별 진행 체크리스트

> 각 단계마다 **성공 판정**이 있다. 판정 통과 못 하면 다음 단계로 넘어가지 말 것.
> 이전 단계가 덜 된 채로 진행하면 원인 파악이 몇 배로 어려워진다.

**공통 준비 — 매번 새 터미널 열 때마다**

```bash
source /opt/ros/humble/setup.bash
source ~/hanium_ws/install/setup.bash
```

**Gazebo 완전 종료 (단계 전환 시 반드시)**

```bash
pkill -9 gzserver; pkill -9 gzclient; pgrep -a gz
```

마지막 `pgrep`에서 아무것도 안 나와야 정상. GUI 창을 닫아도 `gzserver`가 남는 일이 잦고,
그 상태로 다음 단계를 돌리면 CPU를 두 배로 쓰면서 "왜 느리지"가 된다.

---

## STEP 0 — 센서 수정 적용하고 FPS 확보

```bash
cd ~/hanium_ws
pkill -9 gzserver; pkill -9 gzclient
rsync -rt --delete --exclude='__pycache__' --exclude='*.pyc' \
  /media/sf_hanium/hanium_ws/src/ ~/hanium_ws/src/
colcon build --packages-select smart_upcare smart_upcare_control
source install/setup.bash
ros2 launch smart_upcare_control gazebo_4bed.launch.py
```

Gazebo 창을 **전체화면 말고 절반 크기(약 1000×600)** 로 줄인다.
llvmpipe는 CPU로 픽셀을 칠하므로 FPS가 픽셀 수에 거의 반비례한다. 공짜로 얻는 2배.

**성공 판정**

- [ ] FPS 20 이상 (창 축소 상태, 마우스 안 움직이고 안정된 값)
- [ ] Real Time Factor 0.8 이상
- [ ] `ros2 topic list | grep -E "scan|odom"` → `/scan`, `/odom_raw`, `/odometry/filtered` 나옴
- [ ] `ros2 topic hz /scan` → 약 10 Hz

**안 되면**

| 증상 | 대응 |
|---|---|
| FPS는 올랐는데 RTF가 낮음 | LiDAR 부하. `lidar_samples:=180` 으로 더 낮춰본다 |
| FPS가 여전히 10 미만 | 창을 더 줄인다. 그래도 안 되면 헤드리스(`gui:=false`)로 SLAM만 돌리고 녹화는 RViz 중심으로 전환 |
| `/scan` 이 없음 | 로봇 스폰 실패. 터미널 에러 확인 |

---

## STEP 1 — SLAM으로 지도 만들고 저장

STEP 0의 Gazebo는 **켜둔 채로** 진행한다.

```bash
# 터미널 2 — 지도 그리기
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true base_frame:=base_footprint
```

```bash
# 터미널 3 — RViz (지도 그려지는 것 보면서 운전)
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

RViz 설정: Fixed Frame을 `map` 으로, Add → Map (토픽 `/map`), Add → LaserScan (토픽 `/scan`)

```bash
# 터미널 4 — 키보드 운전
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_smoothed -p use_sim_time:=true
```

**운전 요령**

- 속도를 먼저 낮춘다. `teleop`에서 `x` 를 몇 번 눌러 0.2 m/s 근처로.
  빠르면 스캔 정합이 틀어져 지도가 겹쳐 그려진다.
- 경로: 간호 스테이션 → 복도 직진 → **알코브(x≈4) 근처에서 잠깐 정지** →
  계속 직진 → 병실 진입 → 병실 안을 ㄹ자로 한 바퀴 → 다시 복도로 나와 스테이션 복귀
- 알코브·리세스 앞에서 잠깐 멈추면 그 특징이 지도에 또렷하게 박힌다.
- 회전은 천천히. 제자리 회전이 지도를 가장 잘 망가뜨린다.

```bash
# 다 돌았으면 (teleop 끄고 터미널 4에서)
ros2 run nav2_map_server map_saver_cli -f $HOME/hospital_4bed_map \
    --ros-args -p use_sim_time:=true
```

**성공 판정**

- [ ] `~/hospital_4bed_map.pgm` 과 `.yaml` 생성됨
- [ ] 이미지 뷰어로 pgm 열었을 때 평면도와 형태가 일치
- [ ] 복도가 **한 겹의 직선**으로 그려짐 (두 겹으로 겹쳐 보이면 실패 → 다시)
- [ ] 알코브·리세스 요철이 지도에 보임
- [ ] 침상 4개가 사각형 덩어리로 구분됨

**지도가 겹쳐 그려졌다면** 속도를 더 낮추고 처음부터 다시. 이 단계에서 지도가 나쁘면
STEP 2는 절대 안 된다. 여기서 시간을 쓰는 게 훨씬 싸다.

---

## STEP 2 — Nav2 자율주행 (가장 불확실한 단계)

slam_toolbox는 끄고, Gazebo는 켜둔 채로.

```bash
# 터미널 2 (slam_toolbox 종료 후)
ros2 launch smart_upcare_control navigation.launch.py \
    use_sim_time:=true map:=$HOME/hospital_4bed_map.yaml
```

RViz에서:

1. **2D Pose Estimate** 로 로봇의 현재 위치(간호 스테이션, +X 방향)를 찍어 초기 위치를 알려준다
2. LaserScan이 지도의 벽과 겹쳐 보이면 위치추정 성공
3. **2D Goal Pose** 로 목표를 찍는다

**목표 좌표 (터미널로 정확히 보내려면)**

```bash
# Bed 1 옆 — 침대를 바라보는 자세
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "map"}, pose: {position: {x: 10.2, y: 1.0, z: 0.0},
 orientation: {z: 0.7071, w: 0.7071}}}'
```

| 목표 | x | y | orientation z / w |
|---|---|---|---|
| Bed 1 옆 | 10.2 | +1.0 | 0.7071 / 0.7071 |
| Bed 2 옆 | 12.8 | +1.0 | 0.7071 / 0.7071 |
| Bed 3 옆 | 10.2 | −1.0 | −0.7071 / 0.7071 |
| Bed 4 옆 | 12.8 | −1.0 | −0.7071 / 0.7071 |
| 간호 스테이션 복귀 | −0.7 | 0.0 | 1.0 / 0.0 |

**성공 판정**

- [ ] 초기 위치 설정 후 LaserScan이 지도 벽과 일치
- [ ] 경로(초록 선)가 복도를 통과해 병실까지 생성됨
- [ ] 로봇이 복도에서 벽에 안 붙고 중앙으로 주행
- [ ] 침상 옆 목표점에 도착하고 "Goal succeeded"

**막힐 때 — 이 순서로만 손댈 것 (한 번에 하나씩)**

1. **경로가 아예 안 생김** → costmap 문제.
   RViz에서 global costmap을 켜서 복도가 통째로 빨갛게 칠해졌는지 본다.
   그렇다면 `nav2_care.yaml` 의 `inflation_radius: 0.34` 를 0.25로 낮춘다.
   (복도 편측 여유가 0.374 m 라서 0.34는 이론상 통과지만 실측 지도는 벽이 약간 두꺼워진다)
2. **경로는 생기는데 목표에서 계속 맴돔** → `xy_goal_tolerance` 0.15 → 0.25,
   `yaw_goal_tolerance` 0.15 → 0.30
3. **복도에서 멈추거나 진동** → MPPI critics. `vx_max` 를 0.3 → 0.2 로 낮춘다
4. **그래도 안 되면** 목표를 병실 한가운데(x=11.5, y=0.0)로 바꿔서
   "복도 통과 자체는 되는지"부터 분리해 확인한다

> 지금 `nav2_care.yaml` 은 **옛날 1.2 m 직선 복도 월드 기준으로 튜닝된 값**이다.
> 새 지도에서 한 번에 맞을 가능성은 낮으니, 안 된다고 당황하지 말 것. 정상이다.

---

## STEP 3 — 장애물 회피·재계획

로봇이 병실로 주행하는 도중에 새 터미널에서 투입한다.

```bash
# 병실 중앙 통로 — 양쪽 1.40 m 확보, 가장 안정적
ros2 launch smart_upcare_control spawn_obstacle.launch.py x:=11.5 y:=0.0

# 제거
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'demo_obstacle'}"
```

**복도에 놓을 때는 위치가 정해져 있다**

| 위치 | 결과 |
|---|---|
| x 3.6 ~ 4.4 (알코브 구간) | 우회 가능 — **회피 데모** |
| 그 밖의 복도 | 통과 불가 — **정지/대기 데모** (버그 아님) |

복도 폭 1.2 m 에 장애물 0.4 m 를 놓으면 남는 폭이 0.7 m 인데
로봇 폭이 0.451 m 라 인플레이션까지 감당이 안 된다. 알코브가 있는 구간만 1.8 m 로 넓다.

**성공 판정**

- [ ] 장애물 투입 후 costmap에 장애물이 나타남
- [ ] 경로가 다시 그려지며 우회
- [ ] 로봇이 부딪히지 않고 통과

---

## STEP 4 — 시나리오 자동 실행 + 녹화

수동으로 goal을 찍으면서 녹화하면 원테이크가 안 나온다. 시퀀스를 노드에 맡긴다.

```bash
ros2 run smart_upcare_control demo_scenario --ros-args \
  -p use_sim_time:=true \
  -p entry_xy:="[1.0, 0.0]" \
  -p bedside_xy:="[10.2, 1.0]" \
  -p exit_xy:="[-0.7, 0.0]" \
  -p lift_up_mm:=700.0 -p lift_down_mm:=100.0 -p lift_wait_s:=4.0
```

**녹화 원칙**

- Gazebo와 RViz를 **동시에 띄우지 말 것.** VM에서는 둘 다 느려진다.
  같은 시나리오를 두 번 돌려서 각각 녹화하고 편집에서 나란히 합성한다.
- RTF가 낮아도 괜찮다. 녹화 후 편집에서 배속을 올리면 정상 속도로 보인다.
  **FPS가 낮은 것만 후보정으로 못 살린다.**
- 녹화 툴: `sudo apt install simplescreenrecorder` 또는 GNOME 기본 화면 녹화
  (`Ctrl+Alt+Shift+R`)
- 창 크기를 녹화 전에 고정한다. 중간에 바꾸면 편집에서 프레임이 어긋난다.

**성공 판정**

- [ ] 사람 손 안 대고 스테이션 → 복도 → 침상 → 리프트 → 복귀가 한 번에 진행
- [ ] 녹화 파일에서 화면이 끊기지 않음
- [ ] 리프트 상승 시 카메라 TF가 올라가는 게 RViz에서 보임

---

## 오늘의 현실적 목표

**STEP 2까지 (Nav2 goal 1회 성공) + 그 화면 거칠게라도 녹화 1개 확보.**

여기까지 가면 "자율주행이 된다"는 사실이 확정되고, STEP 3~4는 리스크가 아니라
단순 작업량이 된다. 완벽한 원테이크를 노리다 아무것도 못 건지는 것이 최악이다.
거친 녹화라도 손에 쥐고 있으면 안전망이 된다.
