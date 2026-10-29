# 구동계 캘리브레이션 절차 — 8/7

> **전제**: 바퀴 공중(받침대/박스), UNO 펌웨어 업로드 완료, ESP32 없어도 됨
> **도구**: `tools/calibrate_drive.py` — ROS 없이 UNO 직결
> **소요**: 30~40분
> **근거**: 로드맵 §2-1

---

## 왜 ROS 를 안 쓰나

`serial_bridge_node` 는 항상 FF+PI 폐루프를 돌린다. 그런데 여기서 재려는 건
**"PWM 을 넣으면 몇 rad/s 로 도는가"** — 폐루프가 쓸 재료다. 폐루프가 돌아가는
상태로는 이걸 못 잰다. 그래서 UNO 와 직접 통신하는 개루프 도구를 따로 만들었다.

부수효과로 `colcon build` 없이 돌아간다. 젯슨 빌드가 아직이어도 오늘 시작할 수 있다.

---

## 0. 준비

```bash
pip3 install pyserial

# 포트 확인 — CH340 클론이라 ttyACM0 아님 (구동팀 회신 §6-1)
ls -l /dev/serial/by-id/
ls -l /dev/ttyCH341USB*

# ROS 노드·miniterm 전부 종료할 것 (포트 동시 사용 불가)
```

포트가 안 보이면 구동팀 회신 §6-1 의 3단계(WCH 드라이버 빌드 → `brltty` 제거 → dialout 권한).

**바퀴를 공중에 띄운다.** 무부하 기준값을 먼저 잡고, 조립 후 부하 상태에서 재확인한다.

---

## 1. 감속비 + `enc_sign` — 손으로 굴리기

```bash
python3 tools/calibrate_drive.py enc
```

바퀴를 하나씩 **전진 방향으로 정확히 한 바퀴** 굴린다. 타이어에 마커로 기준점을 찍어두면 편하다.

**보는 것**

| 결과 | 의미 |
|---|---|
| 카운트 ≈ **4320** | 감속비 90:1 ✅ 코드 가정과 일치 |
| 카운트 ≈ 1440 | 감속비 30:1 → `mecanum_kinematics.py` 의 `GEAR_RATIO` 수정 필요 |
| 부호 + | `enc_sign = +1` |
| 부호 − | `enc_sign = -1` |

> **감속비가 3배 틀리면 오도메트리 스케일이 통째로 틀어진다.** 조립 전에 확인해두면
> 조립 당일 리스크가 하나 사라진다.

굴리지 않은 바퀴의 카운트가 같이 변하면 **엔코더 배선 교차**다. 스크립트가 잡아준다.

---

## 2. `wheel_sign` — 저속 구동

```bash
python3 tools/calibrate_drive.py motor
```

바퀴별로 +PWM 120 을 1.2초씩 넣고, 그 바퀴가 전진 방향으로 도는지 본다.

**구동팀 회신 §6-2 기준 예상값: `[1, 1, -1, -1]`** (우측 2바퀴 배선 반대)

`enc` 를 먼저 돌려야 한다. `enc_sign` 이 있어야 "raw 부호"에서 "실제 회전 방향"을 역산할 수 있다.

### ★ 두 부호가 다르면

스크립트가 이렇게 알려준다:

```
★ M3(뒤우/RR), M4(앞우/FR) 는 wheel_sign ≠ enc_sign 입니다.
```

**모터 극성과 엔코더 극성이 따로 논다**는 뜻이다. 모터 전원선만 바꿔 끼면 모터 방향은
뒤집히는데 엔코더 A/B 는 그대로이기 때문. 8/7 에 `enc_sign` 파라미터를 분리해둬서 이제는
흡수되지만, 분리 전이었다면 이 두 바퀴가 폐루프에서 **최대속도로 폭주**했을 조합이다.

이 결과가 나오면 구동팀에 `ENC_SIGN[4]` 복원을 제안할 근거가 된다 (선택사항).

---

## 3. `ff_gain` + `deadband_pwm` — 계단 입력

```bash
python3 tools/calibrate_drive.py step
```

PWM 20 → 255 까지 10단계로 올리며 정상상태 각속도를 잰다. 네 바퀴가 동시에 돈다.

**나오는 값**

| 값 | 의미 | 코드 기본값 |
|---|---|---|
| `deadband_pwm` | 이 미만은 정지마찰을 못 이겨 안 돎 | 0 (미측정) |
| `ff_gain` | 목표 각속도 → PWM 환산 계수 | 40.6 (이론 추정치) |
| `max_wheel_omega` | 실측 최대 각속도 | 6.283 (정격 60rpm 가정) |

**바퀴별 편차가 20% 를 넘으면** 스크립트가 경고한다. 모터 개체차가 크면 횡이동(strafe) 시
사선으로 밀리고, PI 게인으로도 완전히는 못 잡는다. 그땐 `ff_per_wheel` 을 바퀴별로 따로
넣는 걸 검토해야 한다.

> `AFMS.begin(100)` — PWM 주파수가 100Hz 로 낮은 편이라 데드밴드가 꽤 클 수 있다.
> 0 이 아니라 20~40 근처가 나와도 이상하지 않다.

---

## 4. 워치독 확인

```bash
python3 tools/calibrate_drive.py watchdog
```

돌리다가 명령을 끊고 1.5초 뒤에 멈춰 있는지 본다. 펌웨어 `CMD_TIMEOUT_MS = 300` 검증.

**이중 방어 구조**라 둘 다 확인해야 한다.

| 계층 | 무엇이 끊길 때 | 값 |
|---|---|---|
| 펌웨어 | 젯슨 → UNO 명령 끊김 | 300ms |
| 젯슨 `cmd_timeout` | Nav2 → 브리지 명령 끊김 | 300ms |
| 젯슨 `enc_timeout` ★8/7 | UNO → 젯슨 엔코더 끊김 | 200ms |

세 번째는 오늘 추가한 것. USB 접촉 불량으로 피드백만 죽으면 PI 적분이 포화해 폭주하는
경로를 막는다. 이건 ROS 를 띄워야 확인되므로 §6 에서 본다.

---

## 5. 결과 정리

```bash
python3 tools/calibrate_drive.py report
```

붙여넣을 수 있는 형태로 나온다.

```
ros2 launch smart_upcare_control drive_test.launch.py \
    --ros-args -p wheel_sign:=[1,1,-1,-1] \
    -p enc_sign:=[1,1,1,1] \
    -p ff_gain:=38.3 \
    -p deadband_pwm:=25
```

값이 안정적이면 `serial_bridge_node.py` 의 `declare_parameter` 기본값으로 굳힌다.
(report 가 그 형태도 같이 출력한다)

---

## 6. ROS 로 넘어가서 확인

여기까지 통과하면 ROS 경로로 올린다.

```bash
colcon build --packages-select smart_upcare smart_upcare_control
source install/setup.bash

ros2 launch smart_upcare_control drive_test.launch.py
```

- WASD 창에서 조종 (W/S 전후진, A/D 횡이동, Q/E 회전, Space 정지)
- **전진 명령에 제자리 회전하면** → `wheel_sign` 이 틀린 것
- **우측 두 바퀴만 최대속도로 튀면** → `enc_sign` 이 틀린 것
- `ros2 topic hz /odom_raw` → 25Hz 근처
- `ros2 topic echo /odom_raw` → 전진 시 `twist.linear.x` 양수, `angular.z` ≈ 0

**엔코더 워치독 확인** — UNO USB 를 뽑는다. 로그에 이게 떠야 한다.

```
[ERROR] 엔코더 업링크 끊김(0.xx s > 0.20 s) → 폐루프 정지. UNO 연결·배선 확인
```

다시 꽂으면 `엔코더 업링크 복구` 가 뜨고 자동으로 재개된다.

---

## 7. 조립 후 다시 할 것

무부하 기준값이라 부하가 걸리면 달라진다.

- [ ] `deadband_pwm` 재측정 — 바닥 마찰 + 로봇 무게로 커진다
- [ ] `ff_gain` 재측정 — 부하로 같은 PWM 에서 각속도가 떨어진다
- [ ] `wheel_sign` / `enc_sign` 재확인 — 배선을 다시 했다면
- [ ] **직진 2m 실측 vs 오도메트리** → `wheel_radius` 보정 (CAD 외경 80.29mm 는 하중 시 눌림)
- [ ] 제자리 90°/180° 회전 → `track_width`/`wheelbase` 검증

---

## 문제 생기면

| 증상 | 확인 |
|---|---|
| 포트를 못 엶 | ROS 노드/miniterm 종료했는지, `dialout` 권한, WCH 드라이버 |
| `<E,..>` 가 안 옴 | 펌웨어 업로드 여부. `python3 -m serial.tools.miniterm /dev/ttyCH341USB0 115200` 로 직접 확인 |
| 카운트가 가끔 크게 튐 | `QGPMaker_Encoder.h` 의 `read()` 인터럽트 미보호 (아래) |
| 한 바퀴만 안 돎 | 모터 배선, PWM 이 데드밴드 미만, 모터 드라이버 채널 |
| 안 굴린 바퀴 카운트가 변함 | 엔코더 배선 교차 |

### 카운트 튐에 대해 — 구동팀 전달용 근거

`QGPMaker_Encoder.h` 의 `position` 은 `volatile` 이 아니고, `read()` 에 인터럽트
보호가 없다. AVR 은 8비트라 32비트 값을 4번에 나눠 읽는데 그 사이에 PCINT 가 끼면
**찢어진 값**이 나온다.

계산하면 바퀴 1rev/s 기준 대략 10초에 한 번, 256 카운트(≈9 rad/s — 정격 6.28 초과)가
튄다. 폐루프에 그대로 들어간다.

`step` 모드 표에서 특정 단계만 값이 크게 벗어나면 이 현상일 가능성이 높다.
**실측으로 잡히면 그 로그와 함께 구동팀에 전달**하는 게 설득력 있다.

수정은 한 줄이다.

```cpp
inline int32_t read() {
  noInterrupts();
  int32_t v = encoder.position;
  interrupts();
  return v;
}
```

---

## 부록: 도구 검증 방법

`tools/test_calibrate_drive.py` 는 pty(가상 시리얼 포트) 위에 UNO 펌웨어를 흉내내는
가짜 장치를 띄우고, **정답을 미리 심어둔 뒤 스크립트가 그걸 되찾는지** 확인한다.

```bash
python3 tools/test_calibrate_drive.py
```

가짜 UNO 에 실차 조건(우측 2바퀴 배선 반대 + 엔코더 정상)을 그대로 넣어뒀다.
실물에서 이상한 값이 나올 때 "도구가 이상한지 로봇이 이상한지"를 가르는 데 쓴다.
