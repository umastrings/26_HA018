# ROS2 / Nav2 디버깅 노트 — 스스로 원인을 찾는 법

> 2026-08-03~04 시뮬레이션 작업에서 실제로 만난 7개 버그를 패턴으로 정리.
> 다음에 비슷한 증상을 만났을 때 **어디부터 볼지** 알기 위한 문서.

---

## PART 1. 문제를 만났을 때의 사고 순서

### 원칙 1 — "안 된다"를 "어디까지 되고 어디서 끊기나"로 바꾼다

"로봇이 안 움직여"는 진단이 아니다. 데이터가 흐르는 경로를 끝에서 거꾸로 짚는다.

```
목표 발행 -> bt_navigator 접수 -> planner 경로 생성 -> controller 속도 계산
  -> collision_monitor 통과 -> planar_move/serial_bridge -> 바퀴
```

각 단계를 **하나씩 확인**해서 처음 끊기는 곳을 찾는다.
한 번에 여러 개를 고치면 뭐가 효과였는지 영영 모른다.

### 원칙 2 — 로그는 "에러"만 보지 않는다

오늘 결정적 단서는 `[ERROR]` 가 아니라 `[INFO]` 였다.

```
[controller_server]: Reached the goal!     <- INFO 인데 명백히 틀린 말
```

**"말이 안 되는 정상 메시지"가 에러보다 중요할 때가 많다.**
로봇이 1mm 도 안 움직였는데 "도착했다"고 하면, 그건 거리 계산 자체가
깨졌다는 뜻이다.

### 원칙 3 — 숫자를 직접 읽는다

로그에 숫자가 나오면 그 숫자가 **무엇의 단위인지** 확인한다.

```
Data time: 1785821239s  /  Transform time: 145s
```

1785821239 는 1970년부터의 초(에포크), 145 는 시뮬 시작부터의 초.
**단위가 다른 두 값을 비교하고 있다** = 시간 기준이 섞였다.
이 한 줄이 이틀치 문제의 원인이었다.

### 원칙 4 — "설정이 적용됐는지"를 믿지 말고 확인한다

launch 에 인자를 줬다고 노드가 그걸 받았다는 보장이 없다.

```bash
ros2 param get /controller_server use_sim_time      # 직접 확인
```

오늘 `use_sim_time:=true` 를 계속 줬지만 노드는 전부 False 였다.

### 원칙 5 — 계산으로 미리 걸러낸다

폴리곤 반폭 0.73 m 인데 복도 반폭이 0.60 m 면, 실행해보지 않아도
**항상 벽을 물 수밖에 없다.** 숫자를 종이에 적어 비교하는 습관이
실행 한 번보다 빠를 때가 많다.

---

## PART 2. 계층별 진단 명령

문제가 생기면 **아래에서 위로** 올라가며 확인한다.
아래층이 깨졌는데 위층을 고치는 건 시간 낭비다.

### 0층 — 프로세스가 깨끗한가

```bash
pgrep -a gz                    # Gazebo 좀비 확인
ros2 node list                 # 유령 노드 확인
ros2 daemon stop && ros2 daemon start   # 노드 정보 캐시 초기화
```

> 부분 재시작은 시간 기준이 어긋난 좀비를 남긴다.
> Gazebo 를 다시 띄울 때는 **전부 내리고 전부 다시** 올린다.

### 1층 — 시간

```bash
ros2 param get /<노드> use_sim_time          # 전부 True 여야 함
timeout 3 ros2 topic echo /clock --once      # 시뮬 시간 확인
```

> ROS2 에서 시간이 어긋나면 TF, costmap, 위치추정이 전부 조용히 실패한다.
> **가장 먼저 확인할 것.**

### 2층 — TF

```bash
ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true
ros2 run tf2_ros tf2_echo odom base_footprint --ros-args -p use_sim_time:=true
python3 /media/sf_hanium/hanium_ws/tools/tf_lag_check.py   # 지연 일괄 측정
```

체인이 이어져야 한다: `map -> odom -> base_footprint -> (센서 링크)`

### 3층 — 센서

```bash
timeout 5 ros2 topic hz /scan
timeout 5 ros2 topic hz /odometry/filtered
```

### 4층 — 노드 상태

```bash
for n in map_server amcl controller_server planner_server bt_navigator \
         behavior_server collision_monitor; do
  printf "%-20s " $n; ros2 lifecycle get /$n 2>/dev/null || echo "(없음)"
done
```

> Nav2 노드는 **active 가 되어야 비로소 토픽 구독을 만든다.**
> "구독자가 없다"는 대부분 "아직 active 가 아니다"라는 뜻이다.

### 5층 — 데이터 흐름

```bash
timeout 5 ros2 topic hz /plan              # 경로 생성됐나
timeout 5 ros2 topic hz /cmd_vel           # 컨트롤러 출력
timeout 5 ros2 topic hz /cmd_vel_smoothed  # collision_monitor 통과 후
gz model -m smart_upcare -p                # 로봇이 실제로 움직이나 (정답지)
```

> `gz model` 은 시뮬레이터가 직접 알려주는 **진짜 위치**다.
> ROS 쪽이 뭐라고 하든 이게 안 변하면 안 움직인 것이다.

---

## PART 3. 오늘 만난 7개 버그

각 항목: **증상 → 근거 → 메커니즘 → 수정 → 일반화**

---

### 버그 1. planar_move 가 잘못된 토픽을 구독

- **증상** teleop 으로 `/cmd_vel_smoothed` 에 보냈는데 로봇이 안 움직임
- **근거** `[planar_move]: Subscribed to [/cmd_vel]`
- **메커니즘** URDF 에 `<command_topic>cmd_vel_smoothed</command_topic>` 을 썼는데,
  이건 **ROS1 전용 태그**다. ROS2 의 gazebo_ros_planar_move 는 이 태그를 읽지 않고
  `<ros><remapping>` 만 본다. 그래서 기본값 `cmd_vel` 을 그대로 썼다.
- **수정** `<ros><remapping>cmd_vel:=cmd_vel_smoothed</remapping></ros>`
- **일반화** 플러그인이 **자기가 뭘 구독하는지 로그에 찍는다.**
  설정을 썼다고 믿지 말고 로그의 "Subscribed to" 를 읽어라.
  ROS1 예제를 ROS2 에 복붙할 때 자주 나오는 실수다.

---

### 버그 2. launch 인자 대소문자

- **증상** `navigation.launch.py` 실행 즉시 종료
- **근거** `[ERROR] [launch]: name 'false' is not defined`
- **메커니즘** nav2_bringup 내부에 `PythonExpression(['not ', slam])` 이 있다.
  문자열을 이어붙여 **파이썬 코드로 평가**하므로 `not false` 가 된다.
  파이썬에는 `false` 라는 이름이 없다(`False` 여야 한다).
- **수정** `'True'/'False'` 로 정규화해서 전달
- **일반화** `name 'xxx' is not defined` 는 거의 항상
  **문자열이 코드로 평가되는 지점**이다. YAML/launch 의 true/false 대소문자를 의심하라.

---

### 버그 3. 파라미터 타입 불일치

- **증상** collision_monitor 가 lifecycle configure 에서 실패, bringup 중단
- **근거** `is of type {double_array}, setting it to {string} is not allowed`
- **메커니즘** 노드가 `points` 를 숫자 배열로 선언했는데 YAML 에는
  `"[[0.75, 0.73], ...]"` 문자열로 적혀 있었다. ROS 버전에 따라 형식이 바뀐다.
- **수정** `[0.70, 0.32, 0.70, -0.32, ...]` 평평한 숫자 배열로
- **일반화** 에러 메시지가 **기대 타입과 실제 타입을 둘 다 알려준다.**
  그대로 읽으면 답이 나온다. 인터넷 예제를 복사할 때 ROS 버전을 확인하라.

---

### 버그 4. 물리적으로 불가능한 설정값

- **증상** 복도에서 로봇이 계속 감속, 결국 `Goal failed`
- **근거** `Robot to slowdown for 30.000000 percents due to PolygonSlow polygon`
- **메커니즘** 감속 폴리곤 반폭 0.73 m > 복도 반폭 0.60 m.
  **폴리곤이 벽보다 넓으니 항상 벽이 안에 들어온다.** 실행 안 해봐도 알 수 있다.
- **수정** 폴리곤을 0.32 로 축소, `slowdown_ratio` 0.3 → 0.5,
  `movement_time_allowance` 10 → 30초
- **일반화** 안전 영역·팽창 반경·통로 폭은 **종이에 그려서 비교**하라.
  로봇 폭 0.451 m, 복도 1.2 m 면 편측 여유는 0.37 m 뿐이다.
  이 숫자를 넘는 설정은 무조건 충돌한다.

---

### 버그 5. 빈 문자열은 "기본값"이 아니다

- **증상** `/goal_pose` 에 `Waiting for at least 1 matching subscription`,
  bt_navigator 는 목록에 있는데 active 가 아님
- **근거** bt_navigator 가 configure 에서 실패
- **메커니즘** `default_nav_to_pose_bt_xml: ""` 를 "기본 BT 쓰라"는 뜻으로 썼는데,
  Nav2 는 **"빈 파일 경로"** 로 해석해 존재하지 않는 파일을 열려다 실패한다.
  configure 실패 → active 못 됨 → **구독 자체를 안 만든다.**
- **수정** 해당 줄을 **삭제** (기본값을 쓰려면 지정하지 않아야 한다)
- **일반화** ROS2 lifecycle 노드는 **active 가 되어야 토픽 구독을 만든다.**
  "구독자 없음"을 보면 통신을 의심하기 전에 `ros2 lifecycle get` 부터 하라.
  그리고 "기본값을 쓰고 싶다"면 빈 값이 아니라 **키를 지우는 것**이 맞다.

---

### 버그 6. 교착 (서로 기다림)

- **증상** Nav2 노드가 통째로 사라짐
- **근거** `global_costmap: Timed out waiting for transform from base_footprint to map`
- **메커니즘**
  - global_costmap 은 configure 단계에서 `map` 프레임이 필요
  - AMCL 은 `/initialpose` 를 받아야 `map -> odom` 을 발행
  - 초기 위치는 사람이 나중에 주므로 → **둘이 서로 기다리다 타임아웃 → 전체 중단**
- **수정** AMCL 에 `set_initial_pose: true` + `initial_pose` 를 미리 넣어
  시작하자마자 TF 를 발행하게 함
- **일반화** "A 가 B 를 기다리는데 B 도 A 를 기다린다" 구조를 의심하라.
  초기값을 하나 박아 넣어 고리를 끊는 것이 정석이다.

---

### 버그 7. use_sim_time 미주입 ★ 오늘의 근본 원인

- **증상** 로봇이 1mm 도 안 움직이는데 `Goal succeeded`
- **근거**
  ```
  [tf_help]: Transform data too old when converting from map to odom
  [tf_help]: Data time: 1785821239s / Transform time: 145s
  [controller_server]: Reached the goal!
  ```
- **메커니즘**
  - `1785821239` = 벽시계(에포크), `145` = 시뮬 시간
  - 노드가 **벽시계 기준으로 시뮬 TF 를 조회** → 17억 초 차이 → 변환 실패
  - 거리 계산 불가 → 컨트롤러가 "도착했다"고 판단 → 즉시 종료
  - AMCL 스캔 폐기, costmap "map 프레임 없음" 도 **전부 같은 원인**
- **왜 안 들어갔나** nav2_bringup 은 `RewrittenYaml` 로 `use_sim_time` 을 주입하는데,
  이 방식은 **파일에 이미 있는 키의 값만 교체**한다. 키가 없으면 추가하지 않는다.
  `nav2_care.yaml` 에는 `use_sim_time` 이 한 줄도 없었다.
- **수정** 모든 노드 블록에 `use_sim_time: False` 키를 명시(값은 launch 가 덮어씀)
- **일반화**
  - **하나의 근본 원인이 서로 무관해 보이는 증상 5~6개를 만든다.**
    증상마다 따로 고치려 들면 끝이 없다. 공통 분모를 찾아라.
  - ROS2 시뮬에서 이상하면 **`use_sim_time` 을 가장 먼저 의심하라.**
  - launch 에 인자를 줬다고 적용된 게 아니다. `ros2 param get` 으로 확인하라.

---

## PART 4. 자주 나오는 로그 문구 사전

| 로그 문구 | 진짜 의미 | 먼저 볼 곳 |
|---|---|---|
| `Waiting for at least 1 matching subscription` | 구독자 없음 | `ros2 lifecycle get` (active 인가) |
| `Invalid frame ID "map" ... does not exist` | 그 프레임 TF 발행자가 없음 | AMCL/slam_toolbox 가 스캔을 받는가 |
| `timestamp ... earlier than all the data in the transform cache` | 메시지와 TF 의 시간 기준이 다름 | `use_sim_time` |
| `Transform data too old` + 숫자 두 개 | 두 시계가 다름 | 숫자 단위를 읽어라 (에포크 vs 시뮬) |
| `is of type {A}, setting it to {B}` | 파라미터 타입 불일치 | 해당 YAML 키 형식 |
| `name 'false' is not defined` | 문자열이 파이썬으로 평가됨 | true/false 대소문자 |
| `Reached the goal!` 인데 안 움직임 | 거리 계산이 깨짐 | 시간/TF |
| `Failed to change state for node` | lifecycle configure 실패 | 그 노드의 파라미터 |
| `Control loop missed its desired rate` | CPU 부족 | 주기를 낮춰라 |
| `Robot to slowdown for N percents` | 안전 폴리곤에 장애물 | 폴리곤 크기 vs 통로 폭 |
| `Entity [X] already exists` | 시뮬이 두 번 떠 있음 | `pgrep -a gz` |

---

## PART 5. 다음에 막혔을 때의 체크리스트

```
[ ] 좀비 프로세스 없나            pgrep -a gz / ros2 node list
[ ] use_sim_time 전부 True 인가   ros2 param get
[ ] TF 체인이 이어지나            tf2_echo map odom / odom base_footprint
[ ] 노드가 전부 active 인가       ros2 lifecycle get
[ ] 센서가 나오나                 ros2 topic hz /scan
[ ] 데이터가 어디서 끊기나        /plan -> /cmd_vel -> /cmd_vel_smoothed
[ ] 로봇이 실제로 움직이나        gz model -m smart_upcare -p
[ ] 설정값이 물리적으로 가능한가  통로 폭 vs 로봇 폭 vs 폴리곤 vs 팽창반경
```

**한 번에 하나만 바꾸고, 바꾼 뒤 반드시 확인한다.**
두 개를 동시에 바꾸면 어느 쪽이 효과였는지 알 수 없고,
그 정보를 잃으면 다음에 같은 문제를 또 겪는다.
