# Smart Up-Care — 젯슨 ROS2 패키지 실차 통합 핸드오프

> **From** 구영인 (자율주행/퍼셉션 · 젯슨측 ROS2)
> **To** 실차 통합 담당 (아두이노/ESP32 펌웨어 · 센서 · 배선)
> **대상 코드** `hanium_ws/src/` — `smart_upcare`(URDF/월드), `smart_upcare_control`(런타임 노드)
> **기준일** 2026-07-27

이 문서 하나로 "젯슨이 뭘 기대하고, 펌웨어가 뭘 맞춰야 하는지"를 파악할 수 있게 정리했어.
제일 중요한 건 **§2 시리얼 프로토콜** — 이게 펌웨어 작성 스펙이야.

---

## 1. 역할 경계 (누가 뭘 하나)

```
   [아두이노/ESP32 펌웨어 · 실차]          [젯슨 ROS2 · 이 패키지]
   ─────────────────────────────          ──────────────────────────
   UNO   : 엔코더 카운트, 모터 PWM 인가     serial_bridge_node:
   ESP32#1: BNO055 IMU, 리프트 스텝            - 시리얼 3포트 수신/송신
   ESP32#2: 섀시폭 스텝 (옵션)                - 메카넘 IK/FK
                                              - 속도 폐루프(FF+PI)
   ↕ USB 시리얼 (아래 프로토콜)              - odom/imu/lift TF 파이프라인
                                            + EKF, Nav2, SLAM, Collision Monitor
```

**핵심 전제 — 속도 제어는 젯슨이 한다.**
UNO는 개루프(들어온 PWM을 그대로 `analogWrite`, PID 없음). 젯슨이 엔코더 각속도를
피드백으로 FF+PI를 돌려 **최종 PWM만** 내려보낸다. 그래서 UNO 펌웨어엔 PID가 필요 없고,
받은 PWM을 부호(방향)+크기로 모터에 인가만 하면 됨.

---

## 2. 시리얼 프로토콜 (★ 펌웨어 작성 스펙)

- 공통: `<` `>` 프레이밍, 그 안은 CSV 텍스트, 첫 필드는 타입 마커, 끝에 `\n`
- 포트마다 한 종류만 흐름. 마커는 오파싱 방지 + 디버깅용
- 정의 위치: `smart_upcare_control/smart_upcare_control/serial_protocol.py`

### 2-1. 업링크 (MCU → 젯슨)

| 출처 | 패킷 | 필드 | 비고 |
|---|---|---|---|
| **UNO** | `<E,c1,c2,c3,c4>` | c1~4 = 누적 엔코더 카운트(long, 부호 포함) | **델타 아님, 누적**. 유실돼도 다음 패킷서 복구. 목표 20~30Hz |
| **ESP32 #1** | `<I,yaw,gyro_z,lift_mm>` | yaw[rad, CCW+], gyro_z[rad/s], lift_mm[mm] | BNO055 heading + 리프트 현재높이 |
| **ESP32 #2** | `<W,width_mm>` | 현재 섀시폭[mm] | 옵션·비블로킹 (없어도 됨) |

휠 순서(엔코더·PWM 공통): **c1=M1 앞좌(FL), c2=M2 뒤좌(RL), c3=M3 뒤우(RR), c4=M4 앞우(FR)**
baud: UNO/ESP 모두 115200 기준 (조정 가능, 젯슨 파라미터와 맞추면 됨)

예: `<E,12034,-5671,8890,-2234>` / `<I,0.7854,0.1200,120.5>` / `<W,394.0>`

### 2-2. 다운링크 (젯슨 → UNO)

| 대상 | 패킷 | 필드 | 비고 |
|---|---|---|---|
| **UNO** | `<M,pwm1,pwm2,pwm3,pwm4>` | 바퀴별 PWM 정수 **−255~+255** | 부호=방향. 50Hz 발행 |

예: `<M,180,-180,180,-180>`

> **확인 필요 ①** UNO의 `Setspeed 16× 스케일`이 펌웨어 안에 있는지.
> 젯슨은 **최종 PWM(−255~255)** 을 보낸다고 가정. UNO가 별도 스케일을 또 곱하면 이중 적용됨.

> **확인 필요 ② (300ms 워치독)** 젯슨은 명령이 끊기면 `<M,0,0,0,0>`을 보내고,
> 자체적으로도 목표를 0으로 만듦. **펌웨어에서도** 마지막 다운링크 수신 후 300ms 넘으면
> 자동 정지하도록 넣어주면 안전(이중 방어). 반영 가능한지 확인.

---

## 3. 토픽 아키텍처 (젯슨 내부)

```
 UNO <E,..> ─┐
 ESP1<I,..> ─┼─▶ serial_bridge_node ─┬─▶ /odom_raw ─┐
 ESP2<W,..> ─┘   (IK/FK + FF/PI)     ├─▶ /imu/data ─┼─▶ EKF ─▶ /odometry/filtered
                                     └─▶ /lift_mm   ┘        (odom→base_footprint TF 소유)
                                          │
                          lift_tf_node ◀──┘ ─▶ /joint_states ─▶ robot_state_publisher
                                                                  (리프트 prismatic TF)

 Nav2 controller ─▶ /cmd_vel ─▶ collision_monitor ─▶ /cmd_vel_smoothed ─▶ serial_bridge_node
                                (30cm 정지 / 50cm 감속)                    ─▶ <M,..> 다운링크
```

- **TF는 EKF가 소유** (`odom→base_footprint`). serial_bridge는 TF를 발행하지 않고 `/odom_raw`만 냄.
- 로봇이 실제로 움직이는 최종 명령 토픽은 **`/cmd_vel_smoothed`** (Collision Monitor 통과분).

---

## 4. 실차 브링업 순서

```bash
# 0) 최초 1회: 의존성 설치 + 빌드
cd ~/hanium_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select smart_upcare smart_upcare_control
source install/setup.bash

# 1) 로봇 브링업 (센서 + 시리얼 브리지 + EKF + TF)
ros2 launch smart_upcare_control robot_bringup.launch.py \
    enable_esp2:=false          # ESP32 2대 확보 시 true

# 2) 네비게이션 (SLAM으로 맵 만들 때 slam:=true, 데모는 저장맵+AMCL)
ros2 launch smart_upcare_control navigation.launch.py \
    slam:=false map:=~/care_map.yaml

# 3) 데모 시퀀스
ros2 launch smart_upcare_control demo_scenario.launch.py
```

> 시뮬(Gazebo)은 `robot_bringup` 대신 `smart_upcare/spawn.launch.py` + `ekf_sim.launch.py`를 씀.
> `serial_bridge`/`sensors`/`robot_bringup`은 **실차 전용**.

---

## 5. 조립 후 교체할 실측값 (TODO)

전부 파라미터 + 코드 내 `TODO` 주석으로 빼뒀어. **파일 수정 없이** launch 인자나 파라미터로 교체 가능.

| 값 | 어디서 | 현재 기본값 | 확정 방법 |
|---|---|---|---|
| 시리얼 포트 3개 | `robot_bringup` / serial_bridge 파라미터 | `/dev/ttyACM0`,`ttyUSB0/1` | `/dev/serial/by-id/…` 고정경로로 |
| `track_width`,`wheelbase` | launch 인자 (URDF+IK 동시 주입) | **0.20699 / 0.19205** (STEP 실측 7/29) | 조립 후 직선주행으로 재확인 |
| 휠 지름/반경 | mecanum_kinematics, URDF | **80.29mm / 0.040145m** (CAD 외경) | ★조립 후 직선 2m 주행으로 유효반경 보정 |
| `wheel_sign` | serial_bridge 파라미터 | [1,1,1,1] | 바퀴별 전진 시 카운트 증가 확인 |
| `ff_gain`,`deadband_pwm` | serial_bridge 파라미터 | 40.6 / 0 | PWM↔rad/s 계단입력 실측 맵 |
| LiDAR 마운트 높이/좌표/yaw | URDF xacro 인자 | 🔶 0.175m / yaw 35.62° | **7/30 프린트 실측 대기**. yaw는 35.62°+90°×k |
| 카메라 틸트 | URDF xacro 인자 | 0.349rad(20°) | 15~30° 범위서 고정 |

> **치수 근거**: `lx+ly`(=0.19952)와 `distance_per_pulse`(=5.8389e-5)는 위 트랙폭·휠베이스·휠지름에서 **자동 계산**되게 짜둠 → 세 값만 바꾸면 회전 오도메트리까지 정합. 기존 244.4mm 트랙폭은 바퀴 바깥면 오측이었음(정정 완료).

---

## 6. 팀원(펌웨어/실차) 확정 필요 — 블로킹 순

1. **🔴 프로토콜 정합** — §2의 패킷 포맷·필드순서·마커·PWM 범위가 펌웨어와 일치하는지. 안 맞으면 어느 쪽을 맞출지.
2. **🔴 리프트 명령 다운링크 (미정)** — 지금 프로토콜엔 리프트를 **올리는 명령**이 없음. `lift_mm`은 현재높이 업링크만 있음. ESP32#1에 리프트 목표를 내려보내는 패킷(예: `<L,target_mm>`)을 정하면, 젯슨 쪽 `/lift_cmd_mm`(데모가 이미 발행 중)을 거기 연결할게. **효선 언니 패킷 포맷과 함께 확정 필요.**
3. **🟡 `lift_mm` 기준면** — 지면 기준 절대높이인지, 접힘 기준 변위인지. `lift_tf_node` 파라미터(`lift_mm_is_absolute`)만 바꾸면 됨.
4. **🟡 ESP32 대수** — 1대뿐이면 IMU+리프트+섀시폭을 1대에 통합하거나 섀시폭을 고정으로 강등. `enable_esp2`로 제어.
5. **🟡 300ms 워치독** — 펌웨어 반영 가능 여부(§2-2 확인필요②).

---

## 7. 파일 인벤토리

**smart_upcare_control (런타임)**
- `serial_bridge_node.py` — ★ 3포트 시리얼 + IK/FK + FF/PI 폐루프 + 워치독
- `serial_protocol.py` — 패킷 정의(펌웨어 스펙 원본)
- `mecanum_kinematics.py` — 메카넘 IK/FK + 엔코더 상수(감속비 90:1, CPR 4320)
- `lift_tf_node.py` / `virtual_lift_publisher.py` — 리프트 Dynamic TF / 하드웨어 없이 모사
- `demo_scenario_node.py` — 데모 시퀀스(웨이포인트→리프트→180°회전→퇴출)
- `ekf_compare.py` — EKF 진단
- `config/` — `ekf_real`·`ekf_mecanum`·`nav2_care`·`collision_monitor`·`laser_filter`
- `launch/` — `robot_bringup`·`sensors`·`navigation`·`demo_scenario`·`ekf_sim`·`ekf_test`

**smart_upcare (기술서/시뮬)**
- `urdf/smart_upcare.urdf.xacro` — 섀시·센서·리프트(prismatic) 파라미터화
- `worlds/care_facility.world` — 케어시설 복도 시뮬 월드
- `launch/spawn.launch.py` — Gazebo 스폰

> 참고: 예전 `mecanum_ik_node.py`/`mecanum_odom_node.py`는 serial_bridge에 통합되어 삭제됨.
> IK/오도메트리는 이제 전부 `serial_bridge_node.py` 안에 있음.
