# TMC2209 + BNO055 리프트 제어 시스템

ESP32를 이용하여 **TMC2209 스테퍼 모터 기반 리프트의 위치를 제어**하고, **BNO055 IMU 데이터를 동시에 측정·전송**하는 통합 제어 프로그램이다.

시스템은 하드웨어 타이머를 이용하여 모터 STEP 펄스를 생성하므로, 메인 루프에서 BNO055를 읽거나 시리얼 명령을 처리하는 동안에도 일정한 주기로 모터를 구동할 수 있다.

---

## 1. 주요 기능

* TMC2209 기반 스테퍼 모터 제어
* 하드웨어 타이머 기반 STEP 펄스 생성 (25 µs 틱)
* 최고 속도 STEP 주기 **450 µs**
* 가위 각도·가감속에 따른 **속도 프로파일** (10/20)
* HOME / TOP 리미트 스위치 안전 제어
* 전원 인가 후 자동 HOME 하지 않음 — **ROS에서 `H` 명령으로 Homing**
* HOME 위치를 STEP `0`으로 설정
* `H`, `M`, `T` 3단계 명령 제어 + `S` 정지 + `K` keepalive
* `M` 위치 = **30,000 STEP**
* `T` 위치 = **75,000 STEP**
* keepalive 1초 끊김 / 이동 45초 초과 시 자동 정지
* BNO055 Yaw, Z축 각속도 측정
* IMU·리프트 상태 **100 ms(10 Hz)** 주기 전송
* HOME 확인 전 M/T 이동 차단

---

# 2. 핀맵

## BNO055

| 기능     | ESP32 GPIO |
| ------ | ---------: |
| SDA    |     GPIO 8 |
| SCL    |     GPIO 9 |
| I2C 주소 |     `0x28` |

BNO055 객체는 `0x28` 주소로 생성된다.

## TMC2209

| 기능   | ESP32 GPIO |
| ---- | ---------: |
| EN   |    GPIO 13 |
| STEP |    GPIO 18 |
| DIR  |    GPIO 12 |

EN은 LOW에서 활성화되도록 설정되어 있다.

## 리미트 스위치

| 기능   | ESP32 GPIO | 상태       |
| ---- | ---------: | -------- |
| HOME |     GPIO 4 | 눌림 = LOW |
| TOP  |     GPIO 5 | 눌림 = LOW |

두 스위치는 `INPUT_PULLUP`으로 설정되어 있으며, 스위치를 누르면 GND와 연결되어 LOW가 된다.

---

# 3. 모터 제어 방식

모터는 `delayMicroseconds()` 방식이 아니라 **ESP32 하드웨어 타이머의 인터럽트(ISR)** 를 이용하여 STEP 신호를 생성한다.

하드웨어 타이머는 1 MHz(**1 tick = 1 µs**)로 설정되고, 인터럽트는 **25 µs(`TICK_US`)** 마다 발생한다.

ISR은 `halfTicks` 틱마다 한 번 STEP 핀을 토글한다.

```text
halfTicks = BASE_HALF_TICKS × factor
BASE_HALF_TICKS = 225 µs / 25 µs = 9
```

`factor = 1`이 최고 속도이며, 이때 STEP 핀은 다음과 같이 동작한다.

```text
225 µs HIGH
      ↓
225 µs LOW
      ↓
1 STEP 완료   (450 µs / STEP)
```

`factor`가 커지면 그만큼 STEP 주기가 늘어난다(느려진다).

---

# 4. 속도 프로파일 (10/20)

`loop()`에서 2 ms마다 `updateSpeed()`가 `halfTicks`를 다시 계산한다. `factor`는 아래 두 값 중 큰 값이다.

### 가위 각도 감속

HOME 근처에서는 가위 다리가 거의 누워 있어 STEP당 필요한 힘이 크다(힘 ∝ 하중 / tan θ). 그래서 다리 각도가 **20° 미만**이면 속도를 줄이며, 최대 **4배**까지 느려진다.

다리 각도는 슬라이더 위치(STEP)로부터 계산한다(`legAngle()`).

### 가감속 램프

출발·도착 전 **1,600 STEP(슬라이더 2 mm)** 구간에서는 최대 **3배** 느린 속도에서 최고 속도까지 선형으로 바뀐다. 플랫폼이 덜컥거리지 않게 하기 위함이다.

### 측정 기하값 (10/20)

| 항목 | 값 |
| --- | ---: |
| 다리 핀-핀 길이 `LEG_MM` | 150 mm |
| 단 수 `STAGES` | 5 |
| HOME 높이 `HOME_HEIGHT_MM` | 65 mm |
| T 높이 | 약 700 mm |
| 리드 스크류 | T8×2, 1/8 마이크로스텝 |
| `STEPS_PER_MM` | 800 |

Homing 중에는 출발 위치를 아는 경우(`homeConfirmed` 상태에서 `H`)에만 각도 감속을 적용한다. 위치를 모르면 램프만 적용한다.

---

# 5. 위치 제어

고정 위치는 두 단계로 설정되어 있다.

```cpp
MID_STEP = 30000;
TOP_STEP = 75000;
```

| 명령  | 동작               |
| --- | ---------------- |
| `H` | HOME 위치로 이동      |
| `M` | 30,000 STEP으로 이동 |
| `T` | 75,000 STEP으로 이동 |

숫자를 직접 입력하는 방식(`P<mm>` 등)은 사용하지 않는다.

---

# 6. Homing

전원을 켜도 **자동으로 HOME 동작을 하지 않는다.** 시작 후 ROS(상위 시스템)가 `H` 명령을 보내야 Homing을 시작한다.

### 이미 HOME에 있는 경우

```text
H 명령
 ↓
HOME 스위치 확인
 ↓
눌림
 ↓
currentStep = 0
 ↓
HOME CONFIRMED
```

### 중간 위치에 있는 경우

```text
H 명령
 ↓
HOME 스위치 확인
 ↓
눌리지 않음
 ↓
HOME 방향으로 이동
 ↓
HOME 스위치 눌림
 ↓
모터 정지
 ↓
currentStep = 0
 ↓
HOME CONFIRMED
```

Homing 중에는 실제 현재 위치를 알 수 없기 때문에 `currentStep`을 증가/감소시키지 않는다. **HOME 스위치가 실제로 눌리는 순간에만 `currentStep = 0`으로 확정한다.**

이 방식으로 전원을 끄기 전에 반드시 HOME 위치로 복귀할 필요가 없다.

---

# 7. HOME 스위치 안전 제어

HOME 방향으로 이동하는 동안 HOME 스위치가 눌리면 하드웨어 타이머 ISR에서 즉시 모터를 정지시킨다. 메인 루프에서도 한 번 더 확인한다.

```text
HOME 스위치 LOW
      ↓
running = false
homing = false
      ↓
STEP 출력 LOW
      ↓
currentStep = 0
      ↓
homeConfirmed = true
```

---

# 8. TOP 스위치 안전 제어

상승 중 TOP 스위치가 눌리면 즉시 모터를 정지시킨다.

```text
상승 중
  ↓
TOP 스위치 LOW
  ↓
모터 정지
  ↓
currentStep = 75000
```

TOP 스위치가 눌린 상태에서는 추가 상승을 허용하지 않는다.

---

# 9. Keepalive / 타임아웃 정지

모터가 움직이는 동안 아래 조건 중 하나라도 맞으면 `stopMotor()`로 정지한다.

| 조건 | 시간 |
| --- | ---: |
| 마지막 `K`(또는 `H/M/T`) 수신 후 | 1초 초과 |
| 이동 명령(`H/M/T`) 시작 후 | 45초 초과 |

따라서 상위 시스템은 이동 중 **1초보다 짧은 주기로 `K`를 보내야** 한다. 시리얼 모니터로 직접 테스트할 때도 마찬가지다.

`S` 명령을 받으면 즉시 정지한다.

---

# 10. HOME 확인 후 M/T 제어

`homeConfirmed` 변수로 원점 설정 여부를 관리한다.

```cpp
volatile bool homeConfirmed = false;
```

전원 ON 직후에는 `false`이며 HOME 스위치를 확인하면 `true`가 된다.

HOME이 확인되지 않은 상태에서는 M/T 명령을 실행하지 않는다.

```text
HOME 미확인
   ↓
M 또는 T
   ↓
이동 차단 (HOME NOT CONFIRMED)
```

---

# 11. BNO055 IMU 처리

BNO055는 I2C를 통해 데이터를 읽는다. 동작 모드는 기본값(NDOF)이다.

측정 데이터:

1. Yaw
2. Z축 각속도(Gyroscope Z)

BNO055의 Yaw는 ROS 방향에 맞게 뒤집고 0~360° 범위로 보정한 뒤 라디안으로 바꾼다.

```cpp
float ros_yaw = 360.0 - event.orientation.x;
float yaw_rad = ros_yaw * DEG_TO_RAD;
float gyro_z_rad = gyro.z() * DEG_TO_RAD;   // deg/s -> rad/s
```

---

# 12. 시리얼 전송 형식

100 ms(10 Hz)마다 두 줄을 보낸다.

### 리프트 상태

```text
<L,currentStep,homeConfirmed,running>
```

예:

```text
<L,30000,1,0>
```

### IMU

```text
<I,yaw_rad,gyro_z_rad,lift_mm,state>
```

예:

```text
<I,2.1876,-0.0004,0.0,M>
```

| 필드 | 설명 |
| --- | --- |
| `yaw_rad` | Yaw [rad] |
| `gyro_z_rad` | Z축 각속도 [rad/s] |
| `lift_mm` | 아직 미사용, 항상 `0.0` (STEP→mm 환산 미적용) |
| `state` | `H` / `M` / `T` / `-` (이동 중이거나 고정 위치가 아니면 `-`) |

그 외 `STOP | STEP = ...`, `HOME CONFIRMED` 같은 사람이 읽는 로그 줄도 함께 출력된다.

---

# 13. 메인 루프와 타이머 인터럽트 역할 분리

### 하드웨어 타이머 ISR (`onTimer()`)

* 25 µs마다 호출
* `halfTicks`마다 STEP 토글
* HOME / TOP 스위치 즉시 정지
* `currentStep` 갱신

### `loop()`

* keepalive / 45초 타임아웃 정지
* 2 ms마다 `updateSpeed()`로 속도 갱신
* 시리얼 명령을 한 글자씩 받아 줄바꿈(`\n`) 단위로 처리 (32자 초과 시 버림)
* HOME / TOP 상태 이중 확인
* 100 ms마다 `<L>`, `<I>` 전송

따라서 BNO055를 읽는 작업이 발생하더라도 모터 STEP 생성은 하드웨어 타이머에 의해 독립적으로 수행된다.

---

# 14. 전체 동작 흐름

```text
                 전원 ON
                    │
                    ▼
              BNO055 초기화
                    │
                    ▼
             하드웨어 타이머 시작
                    │
                    ▼
            대기 (homeConfirmed = false)
                    │
                  H 명령
                    │
          ┌─────────┴─────────┐
          │                   │
     HOME 눌림            HOME 안 눌림
          │                   │
          ▼                   ▼
      STEP = 0          HOME 방향 이동
          │                   │
          │              HOME 스위치
          │                   │
          └─────────┬─────────┘
                    ▼
             HOME CONFIRMED
                    │
          ┌─────────┼─────────┐
          │         │         │
          H         M         T
          │         │         │
          ▼         ▼         ▼
        HOME      30000     75000

   (이동 중 K 1초 끊김 / 45초 초과 / S 명령 → 정지)
```

---

# 15. 명령어 요약

| 입력  | 기능          |          목표 |
| --- | ----------- | ----------: |
| `H` | HOME 이동     |    HOME 스위치 |
| `M` | 중간 위치       | 30,000 STEP |
| `T` | 상단 위치       | 75,000 STEP |
| `S` | 즉시 정지       |           - |
| `K` | keepalive   |           - |
| 그 외 | 무효 명령       |           - |

`H/M/T`는 모터가 이미 움직이는 중이면 `MOTOR RUNNING`을 출력하고 무시한다. `S`, `K`는 대문자만 인식한다.

---

# 16. 주요 파라미터

```cpp
// 핀
EN_PIN    = 13
STEP_PIN  = 18
DIR_PIN   = 12

HOME_PIN  = 4
TOP_PIN   = 5

// 모터
STEP_DELAY_US = 450     // 최고 속도
TICK_US       = 25

// 속도 프로파일
SLOW_BELOW_DEG = 20.0
MAX_SLOW       = 4.0
RAMP_STEPS     = 1600
RAMP_SLOW      = 3.0

// 위치
MID_STEP = 30000
TOP_STEP = 75000

// IMU
IMU_INTERVAL = 100 ms

// 안전
keepalive 타임아웃 = 1000 ms
이동 최대 시간     = 45000 ms
```
