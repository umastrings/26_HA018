# hanium_실차 전 코드 점검 (Phase 0) — 8/7

> **작성 근거**: `hanium_ws/src` 전수 읽기 + `README_실차통합_핸드오프.md`(7/27) ·
> `hanium_시뮬_진단기록_0804.md` · `hanium_하드웨어_완성_로드맵_체크리스트.md` 대조
> **상태**: 일부 조립 완료 · 실차 주행 준비 착수 시점
> **목적**: 실물에서 처음 `robot_bringup` 을 올렸을 때 **터질 것 / 조용히 틀릴 것** 을 미리 잡기

---

## 요약

시뮬 트랙에서 얻은 수정(8/3~8/5)이 **`navigation.launch.py` 계열에만 반영되고
`robot_bringup.launch.py` 계열에는 backport 되지 않았다.** 실차 브링업 경로는
시뮬에서 한 번도 실행된 적이 없는 코드 경로라서, 여기가 위험 구간이다.

| 등급 | 개수 | 성격 |
|---|---|---|
| 🔴 기동 실패 / 물리적 위험 | 5 | 실물에서 확실히 터지거나 벽을 긁음 |
| 🟡 조용히 틀림 | 4 | 기동은 되는데 의도대로 동작 안 함 |
| 📌 문서-코드 불일치 | 3 | 팀원 전달본·심사 설명에 직결 |

---

## 🔴 1. 시리얼 포트 충돌 — 라이다와 ESP32#1이 같은 포트

```
sensors.launch.py      : lidar_port  기본값 /dev/ttyUSB0   (RPLIDAR A2M12)
serial_bridge_node.py  : esp1_port   기본값 /dev/ttyUSB0   (ESP32 #1)
```

`robot_bringup.launch.py` 는 이 둘을 **동시에** 올린다. 먼저 연 쪽이 포트를 잡고
나중 쪽은 `SerialException` → `_open()` 이 `raise` → 브리지 노드 사망.

라이다·ESP32 둘 다 CP210x/CH340 계열이면 꽂는 순서에 따라 `ttyUSB0/1` 이 매번
바뀌므로, 기본값을 바꾸는 것으로는 못 막는다. **`/dev/serial/by-id/` 고정 경로
전환은 "TODO"가 아니라 첫 브링업 전 필수 작업.**

```bash
# 젯슨에서 장치 3~4개 꽂고 실행 → 각 장치의 고정 경로 확보
ls -l /dev/serial/by-id/
```

---

## 🔴 2. `robot_bringup.launch.py` 에 8/4 `use_sim_time` 교훈 미반영

`navigation.launch.py:40-44` 에 직접 남긴 교훈:

> dict 로 LaunchConfiguration 을 그냥 넘기면 문자열 `'true'` 로 전달되어
> 노드가 무시하고 벽시계를 쓰는 경우가 있다.

그런데 `robot_bringup.launch.py` 는 그 패턴을 그대로 쓰고 있다. 4곳:

| 줄 | 파라미터 | 노드가 기대하는 타입 |
|---|---|---|
| 58, 74, 85, 92, 98 | `use_sim_time` | bool |
| 75 | `enable_esp2` | bool |
| 76 | `track_width` | double |
| 77 | `wheelbase` | double |

`robot_description` 만 `ParameterValue(..., value_type=str)` 로 처리돼 있다(48행).
나머지도 같은 방식으로 감싸야 한다.

```python
sim_time_bool = ParameterValue(use_sim_time, value_type=bool)
esp2_bool     = ParameterValue(enable_esp2, value_type=bool)
tw_f          = ParameterValue(track_width, value_type=float)
wb_f          = ParameterValue(wheelbase,  value_type=float)
```

> ⚠️ 특히 `track_width` / `wheelbase` 가 문자열로 들어가면 **IK 상수가 기본값으로
> 남는다.** 조용히 틀리는 쪽이라 더 위험 — launch 인자로 실측값을 바꿔도 반영이 안 됨.

---

## 🔴 3. `pyserial` 의존성이 `package.xml` 에 없음

핸드오프 §4 브링업 0단계가 `rosdep install --from-paths src ...` 인데,
`smart_upcare_control/package.xml` 에 pyserial 항목이 없다.
→ 깨끗한 젯슨에서 `rosdep` 통과 후에도 `import serial` 이 `ModuleNotFoundError`.

같이 빠진 것:

```xml
<exec_depend>python3-pyserial</exec_depend>   <!-- serial_bridge_node -->
<exec_depend>laser_filters</exec_depend>      <!-- config/laser_filter.yaml -->
<exec_depend>topic_tools</exec_depend>        <!-- navigation.launch.py relay 우회 -->
```

---

## 🔴 4. ESP32#1 없으면 브리지 전체가 죽는다 — `enable_esp1` 스위치 부재

`enable_esp2` 는 있는데 `enable_esp1` 은 없다. `_open()` 은 실패 시 `raise` 하고,
`main()` 은 `node = SerialBridgeNode()` 가 `try` **밖**에 있어 그대로 크래시한다.

**지금 상황(일부 조립)에 직결된다.** UNO+바퀴+엔코더만 붙은 상태에서
로드맵 §2-1 캘리브레이션(감속비 실측·`wheel_sign`·PWM↔rad/s 맵)을 하려면
ESP32 없이 브리지가 떠야 하는데, 현재 코드로는 불가능하다.

필요한 것 두 가지:

1. `enable_esp1` 파라미터 (기본 True, 캘리브레이션 시 False)
2. `_open()` 실패를 치명 종료가 아니라 **해당 포트만 비활성** 으로 강등
   (단, UNO 실패는 치명 유지 — 모터 제어 불가이므로)

---

## 🔴 5. `local_costmap` footprint 가 시뮬 편의로 축소된 채 남아 있음

`hanium_시뮬_진단기록_0804.md` §4-1 이 명시적으로 경고한 항목:

```
nav2_care.yaml:325  local_costmap  footprint = 0.34 × 0.30   ← 시뮬 URDF 기준 축소본
nav2_care.yaml:355  global_costmap footprint = 0.499 × 0.451 ← 실물 펼침 기준
```

로컬/글로벌이 서로 다른 로봇 크기를 믿고 있다. 실물 펼침 상태는 394×492mm 이므로
**local_costmap 이 실제보다 15cm 작은 로봇을 가정한 채 회피 경로를 짠다** → 벽 접촉.

0804 문서의 결론대로, 실물에선 되돌리되 "축소폭(280×378) 상태 회전"은
footprint 동적 변경으로 별도 구현해야 한다.

---

## 🟡 6. 엔코더 스트림 stale 검증이 없다

`control_loop()` 에는 `cmd_timeout`(300ms 워치독)이 있지만, **엔코더 쪽 타임아웃은
없다.** `meas_omega` 는 `on_uno()` 에서만 갱신되므로:

> UNO 업링크만 끊기고 `/cmd_vel_smoothed` 는 계속 들어오는 상황
> → `meas_omega` 가 마지막 값에 고정 → 오차가 상수로 굳음
> → 적분항이 계속 쌓여 **PWM 이 한쪽으로 포화**

실물에서 USB 접촉 불량 하나로 로봇이 폭주할 수 있는 경로다. `enc_timeout`
(예: 0.2s) 추가 → 초과 시 PWM 0 + 적분 리셋 + 경고.

---

## 🟡 7. `/scan_depth` 를 아무도 안 쓴다

`sensors.launch.py` 가 `depthimage_to_laserscan` 을 띄워 `/scan_depth` 를 내지만,

- `nav2_care.yaml` : local/global 둘 다 `observation_sources: scan` (→ `/scan` 만)
- `collision_monitor.yaml` : `observation_sources: ["scan"]` (→ `/scan` 만)

즉 **D415 는 현재 Nav2 에 아무 영향을 주지 않는다.** 로드맵 §2-2 의
"depth 를 costmap 에 투입" 이 절반만 된 상태. 문턱·발판 같은 저상 장애물 대응은
이 배선을 마쳐야 성립한다.

---

## 🟡 8. IMU `orientation_covariance[0] = -1.0`

```python
imu.orientation_covariance[8] = 0.01    # yaw 는 신뢰
imu.orientation_covariance[0] = -1.0    # roll/pitch 미사용 의도
```

REP-145 에서 `orientation_covariance[0] == -1` 은 **"orientation 전체를 제공하지
않음"** 을 뜻한다. roll/pitch만 빼려는 의도라면 [0], [4] 에 큰 값(1e6)을 넣는 게 맞다.
`robot_localization` 은 자체 config 행렬을 쓰므로 당장 EKF 는 정상 동작할 가능성이
높지만, RViz2·다른 노드·심사 시연에서 오해를 부른다.

(바로 아래 `linear_acceleration_covariance[0] = -1.0` 은 **올바른** 사용 — 선가속도를
실제로 제공하지 않으므로.)

---

## 🟡 9. 리프트 다운링크가 여전히 끊겨 있음

```
demo_scenario_node  --publish-->  /lift_cmd_mm   →  (구독자 없음)
```

`serial_protocol.py` 에 리프트 다운링크 패킷이 아예 정의돼 있지 않고,
`serial_bridge_node` 에도 `/lift_cmd_mm` 구독이 없다. 7/27 핸드오프 §6-2 의
🔴 항목이 그대로 남아 있다 — **효선 언니 확정 대기 중 최장기 블로커.**

---

## 📌 10. 핸드오프 §3 토픽 체인 다이어그램이 8/5 실측과 다름

핸드오프(7/27) 기재:

```
Nav2 controller → /cmd_vel → collision_monitor → /cmd_vel_smoothed → serial_bridge
```

8/5 실측(`nav2_care.yaml:234-235`):

```
controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel
                  → collision_monitor → /cmd_vel_smoothed → serial_bridge
```

`velocity_smoother` 가 빠져 있다. 이건 **8/4 복도 정지의 진짜 원인이었던 노드**라
(y 성분이 0.0 으로 클램프되어 메카넘 횡이동 전량 차단) 팀 전달본과 심사 설명에서
반드시 정정돼야 한다.

---

## 📌 11. 핸드오프 §3 의 폴리곤 수치가 실제 yaml 과 다름

| | 핸드오프 §3 | 실제 `collision_monitor.yaml` (8/4 재조정) |
|---|---|---|
| stop | 30cm | 전방 0.50 / 후방 0.32 / 좌우 0.28 |
| slowdown | 50cm | 전방 0.70 / 후방 0.40 / 좌우 0.32 |

또 `inflation_radius: 0.34` 는 좌우 stop(0.28)보다는 크지만 **전방 stop(0.50)보다
작다.** "inflation_radius 가 stop 폴리곤 거리를 넘어야 한다"는 원칙이 전방에서는
깨져 있다 — 전방은 Collision Monitor 가 1차 장애물 반응이 된다.
전방 편향 폴리곤을 의도적으로 택한 8/4 결정의 부수효과이므로, **의도된 것인지
재확인 후 문서에 근거를 남길 것.**

---

## 📌 12. `demo_scenario.rotate()` 가 velocity_smoother 를 우회

`/cmd_vel` 에 직접 발행 → velocity_smoother 하류이므로 **가속 제한을 안 받는다.**
시뮬에선 문제없었지만 실물 메카넘에서 0.5 rad/s 계단 입력은 롤러 슬립·급격한
전류 피크를 만든다. `max_accel` 기준으로 램프를 넣거나 `/cmd_vel_nav` 로 보낼지 결정 필요.

(behavior_server 도 같은 방식으로 `/cmd_vel` 에 직접 쏘므로 Nav2 자체 패턴과는 일치.
안전 게이팅은 collision_monitor 를 통과하므로 유지됨.)

---

## 실행 순서 제안

**A. 지금 바로 (코드 수정, 실물 불필요)**

1. §3 `package.xml` 의존성 3줄 추가 — 가장 싸고 젯슨 첫 빌드를 살림
2. §2 `robot_bringup.launch.py` ParameterValue 감싸기
3. §4 `enable_esp1` 추가 + `_open()` 실패 강등 — **캘리브레이션의 전제조건**
4. §6 `enc_timeout` 추가 — 실물 안전
5. §1 포트를 launch 인자로 노출 (by-id 값은 젯슨에서 확인 후 주입)

**B. 젯슨에서 확인 필요 (영인 님)**

- `ls -l /dev/serial/by-id/` → 장치별 고정 경로
- `colcon build --packages-select smart_upcare smart_upcare_control` 통과 여부
- 현재 조립된 범위 (UNO·엔코더·라이다·ESP32 중 무엇이 붙어 있는지)

**C. 팀 확정 대기 (재촉 대상)**

- §9 리프트 다운링크 패킷 — 효선 언니 (최장기 블로커)
- §5 실물 footprint 확정 — 섀시팀 최종 치수

**D. 문서 갱신 (한 번에)**

- §10 §11 핸드오프 정정 → 팀 재배포
