# 4인실 자율주행 시뮬 — 이어서 하기 (2026-08-03 마감 시점)

> 다음 세션에서 이 문서만 보고 바로 이어갈 수 있게 정리.
> **현재 막힌 지점: Nav2 목표는 전달되는데 로봇이 안 움직임.**

---

## 1. 오늘 확보한 것 (전부 파일로 저장됨)

| 항목 | 상태 | 위치 |
|---|---|---|
| 4인실 월드 | 완성, 기하 검증 통과 | `src/smart_upcare/worlds/hospital_4bed.world` |
| 월드 생성기 | 치수 바꾸면 재검증까지 자동 | `worlds/gen_hospital_4bed.py` |
| **SLAM 지도** | **저장 완료. 품질 양호(복도 한 겹)** | `~/hospital_4bed_map.pgm` / `.yaml` |
| 데모 장애물 | 박스 / 휠체어 | `worlds/models/*.sdf` |
| SLAM 설정 | base_frame, 루프클로저 튜닝 | `config/slam_hospital_4bed.yaml` |
| RViz 화면 | 지도·스캔·경로 세팅됨 | `src/smart_upcare/rviz/slam_4bed.rviz` |
| 동기화 스크립트 | 공유폴더 → VM + 빌드 | `~/sync_hanium.sh` |

**성능**: 헤드리스 + `lidar_rate:=5 imu_rate:=50` 에서 RTF 1.06~1.28,
시뮬 스캔 간격 0.2005초(±2%). SLAM/Nav2 에 충분.

---

## 2. 오늘 고친 버그 6개 (전부 "실행된 적 없던 설정"에서 나옴)

| # | 파일 | 문제 | 수정 |
|---|---|---|---|
| 1 | `smart_upcare.urdf.xacro` | `<command_topic>` 은 ROS1 태그라 무시됨 → planar_move 가 `/cmd_vel` 구독 | `<ros><remapping>cmd_vel:=cmd_vel_smoothed</remapping>` |
| 2 | `navigation.launch.py` | `slam:='false'` 소문자 → nav2 내부 `not false` → NameError | `PythonExpression` 으로 `'True'/'False'` 정규화 |
| 3 | `collision_monitor.yaml` | `points` 가 문자열 → 이 Nav2 는 double_array 요구 | 평평한 숫자 배열로 변경 |
| 4 | `collision_monitor.yaml` | 폴리곤 반폭 0.73 m > 복도 반폭 0.60 m → 항상 벽 물음 | 전방 편향으로 축소 (y ±0.35 / ±0.40) |
| 5 | `nav2_care.yaml` | `default_nav_to_pose_bt_xml: ""` → BT 로드 실패 → bt_navigator ACTIVE 불가 | 해당 줄 제거 |
| 6 | `nav2_care.yaml` | AMCL 초기위치 대기 ↔ costmap map 프레임 대기 교착 | `set_initial_pose: true` + `initial_pose` |

추가로 costmap `transform_tolerance: 1.0` (기본 0.3 은 VM 에서 부족).

---

## 3. 현재 도달 지점

```
map_server           active
amcl                 active
controller_server    active
planner_server       active
bt_navigator         active
behavior_server      active
collision_monitor    active
map -> odom          Translation: [-0.040, 0.040, 0.000]   (초기위치 정상)
/goal_pose 발행      구독자 찾음, 전송 성공
로봇 이동            없음  ← 여기서 막힘
```

**즉 "목표는 접수됐는데 바퀴 명령이 안 나가는" 한 칸만 남음.**

---

## 4. 다음 세션 시작 절차 (약 5분)

```bash
# 0) 완전 정리 — 이거 안 하면 "Entity already exists" 등으로 꼬임
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f gazebo
pkill -9 -f component_container; pkill -9 -f ekf_node; pkill -9 rviz2
sleep 3
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
pgrep -a gz          # ★ 아무것도 안 나올 때까지 반복

# 1) 동기화
~/sync_hanium.sh

# 2) 터미널 1 — 시뮬 (여기서만!)
source ~/hanium_ws/install/setup.bash
ros2 launch smart_upcare_control gazebo_4bed.launch.py gui:=false lidar_rate:=5 imu_rate:=50

# 3) 터미널 2 — 물리 시작
source ~/hanium_ws/install/setup.bash
ros2 service call /unpause_physics std_srvs/srv/Empty {}

# 4) 터미널 3 — Nav2
source ~/hanium_ws/install/setup.bash
ros2 launch smart_upcare_control navigation.launch.py \
    use_sim_time:=true map:=$HOME/hospital_4bed_map.yaml 2>&1 | tee /tmp/t3.log

# 5) 40초 뒤 상태 확인
for n in map_server amcl controller_server planner_server bt_navigator behavior_server collision_monitor; do
  printf "%-20s " $n; ros2 lifecycle get /$n 2>/dev/null || echo "(없음)"
done
```

---

## 5. 내일 첫 작업 — 안 움직이는 원인 좁히기

목표를 보낸 직후 **이 순서로** 확인. 처음 끊기는 지점이 원인이다.

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "map"}, pose: {position: {x: 9.18, y: 0.0, z: 0.0},
 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

sleep 3
echo "== 1 경로 =="        ; timeout 5 ros2 topic hz /plan
echo "== 2 컨트롤러 출력 =="; timeout 5 ros2 topic hz /cmd_vel
echo "== 3 최종 명령 =="    ; timeout 5 ros2 topic hz /cmd_vel_smoothed
echo "== 4 CM 상태 =="      ; timeout 4 ros2 topic echo /collision_monitor_state --once
tail -25 /tmp/t3.log
```

| 처음 끊기는 곳 | 원인 | 대응 |
|---|---|---|
| 1 `/plan` 없음 | planner 실패 | 목표가 지도 밖/장애물 위인지 확인. `x:=3.0` 처럼 가까운 목표로 축소 테스트 |
| 2 `/cmd_vel` 없음 | 컨트롤러가 경로를 못 따름 | MPPI `vx_min: 0.0`(후진 차단)이 초기 정렬을 막는지 의심 → 일시적으로 `-0.1` 로 |
| 3 `/cmd_vel_smoothed` 없음 | Collision Monitor 가 정지시킴 | `use_collision_monitor:=false` 로 우회 |
| 4 `action_type: stop` | 폴리곤이 벽/침대를 물음 | 폴리곤 y 를 0.30 으로 더 축소 |

### ★ 세션 마감 직전 확인된 인과 사슬 (최우선 확인 대상)

터미널 3 마지막 로그:

```
[global_costmap]: Timed out waiting for transform from base_footprint to map
  tf error: Invalid frame ID "map" ... frame does not exist
```

그런데 `amcl` 은 active 였고 한때 `map->odom` 도 나왔다. 앞뒤 로그를 합치면:

```
AMCL 이 /scan 을 전부 폐기
   ("the timestamp on the message is earlier than all the data in the transform cache")
  -> AMCL 은 스캔 콜백에서만 map->odom 을 broadcast 하므로 TF 갱신 중단
  -> global_costmap 이 map 프레임을 못 찾음
  -> 경로 계획 불가 -> /plan 없음 -> /cmd_vel 없음 -> 로봇 정지
```

**"timestamp 가 transform cache 보다 과거" 의 유력 원인 = 시뮬 시간 리셋**

Gazebo 를 재시작하면 `/clock` 이 0 부터 다시 시작한다. 그런데 `pkill` 로 일부만
죽고 살아남은 노드(EKF, 이전 component_container, rviz 등)는 **재시작 전의 큰
타임스탬프(800초대) TF 를 버퍼에 그대로 들고 있다.** 이 상태에서 새 스캔이
작은 타임스탬프로 들어오면 정확히 위 메시지가 뜬다.

오늘 부분 재시작을 여러 번 했으므로 이 시나리오와 정확히 부합한다.

**→ 다음 세션 첫 작업: 좀비 노드 0 인 상태에서 완전 새 출발 후 재현되는지 확인**

```bash
# 완전 정리 후 반드시 확인
ros2 node list          # 아무것도 없어야 함 (데몬 제외)
ros2 daemon stop        # ROS2 데몬도 재시작 (캐시된 노드 정보 제거)
ros2 daemon start

# 시뮬 띄운 직후 — 시뮬 시간이 0 근처에서 시작하는지
timeout 3 ros2 topic echo /clock --once

# Nav2 띄운 뒤 — AMCL 이 스캔을 실제로 처리하는지
timeout 8 ros2 topic hz /amcl_pose        # 나오면 정상
timeout 8 ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true
```

`/amcl_pose` 가 계속 나오면 AMCL 이 스캔을 처리하고 있다는 뜻이고,
그러면 map->odom 도 유지되어 costmap 이 살아난다.

**주의: 앞으로는 부분 재시작 금지.** Gazebo 를 다시 띄울 때는 Nav2·RViz·EKF 를
포함해 전부 내리고 순서대로 다시 올릴 것.

---

### 유력 후보 (아직 미확인)

- **MPPI `vx_min: 0.0`** — 후진을 완전히 막아둔 설정. 목표 방향으로 회전하기 전에
  전진만 가능하면 초기 궤적 샘플이 전부 비용 초과로 기각될 수 있다.
  진단용으로 `vx_min: -0.1` 로 잠깐 풀어보고 움직이면 이게 원인.
- **DDS 전송 지연** — 오늘 `RTPS_TRANSPORT_SHM Error` 가 반복됐고,
  한때 costmap 에 235초 밀린 스캔이 도착했다. `/dev/shm` 청소로 개선되는지 관찰.
  재발하면 CycloneDDS 로 전환:
  ```bash
  sudo apt install -y ros-humble-rmw-cyclonedds-cpp
  echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
  # 모든 터미널을 새로 열어야 적용됨
  ```

---

## 6. 목표 좌표 (map 프레임, 보정 완료)

`map = 월드 + (0.58, -0.01)` — 8/3 실측 (`gz model` vs `tf2_echo`)

| 목표 | x | y | orientation z / w |
|---|---|---|---|
| 복도 진입 | 1.57 | 0.04 | 0.0 / 1.0 |
| 병실 진입 | 9.18 | 0.00 | 0.0 / 1.0 |
| **Bed 1 옆** | **10.78** | **+0.94** | 0.7071 / 0.7071 |
| **Bed 2 옆** | **13.38** | **+0.94** | 0.7071 / 0.7071 |
| **Bed 3 옆** | **10.78** | **−0.96** | −0.7071 / 0.7071 |
| **Bed 4 옆** | **13.38** | **−0.96** | −0.7071 / 0.7071 |
| 스테이션 복귀 | −0.14 | 0.04 | 1.0 / 0.0 |

**침대 좌표 산출 근거 (8/4 재계산)** — 이전 표(+1.09 / −1.01)는 폐기.

```
bed_1 월드 중심 y = 2.05, 깊이 0.9  →  복도쪽 모서리 y = 1.60
map 변환 (−0.01)                    →  1.59
로봇 반길이 0.246 (492mm/2, 침대 정면 접근)
목표 여유 0.40 m
→ 로봇 중심 y = 1.59 − 0.246 − 0.40 = 0.944 ≈ 0.94
```

여유 0.40 m 는 `inflation_radius: 0.34` 보다 6 cm 크다. 이전 값(+1.09)은
여유가 0.255 m 라 inflation 안에 목표가 들어가서 컨트롤러가 자주 포기했다.
**단, 0.255 m 여도 "불가능"은 아니었다** — 8/4 실측에서 병실 근처에서
출발했을 때는 성공했다. 결정적 실패가 아니라 **확률적 실패**이므로
한 번의 성공/실패로 판단하지 말 것.

**주의**: `nav2_care.yaml` 의 `initial_pose` 는 스폰 좌표 `(-0.60, 0.00)` 기준
`(-0.04, 0.04)` 로 박혀 있다. 스폰 위치를 바꾸면 이것도 같이 바꿀 것.

---

## 7. 남은 단계

- **STEP 2** (진행 중) — Nav2 로 침상 옆 도착
- STEP 3 — 장애물 회피 (`spawn_obstacle.launch.py x:=11.5 y:=0.0`)
  - 복도 장애물은 알코브 구간(x 3.6~4.4)에서만 회피 가능. 그 밖은 정지 데모용
- STEP 4 — `demo_scenario` 노드로 원테이크 + 녹화
  - Gazebo 와 RViz 는 따로 녹화해서 편집에서 합성 (VM 에서 동시 실행 무리)
  - RTF 낮으면 후보정 배속으로 해결 가능. **FPS 낮은 건 못 살림**

---

## 8. 참고 — 심사 Q&A 용

SLAM 은 **gmapping 이 아니라 slam_toolbox**(그래프 기반, Ceres, 루프 클로저).
복도 알코브/리세스는 8 m 평행 복도의 X 방향 위치 모호성을 없애기 위한
루프 클로저 단서로 **의도적으로 설계**한 것. 기각한 대안 목록
(micro-ROS, ORB-SLAM3)에 gmapping 도 추가해둘 것.
