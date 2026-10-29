 

# TMC2209 + BNO055 리프트 제어 시스템

ESP32를 이용하여 **TMC2209 스테퍼 모터 기반 리프트의 위치를 제어**하고, **BNO055 IMU 데이터를 동시에 측정·전송**하는 통합 제어 프로그램이다.

시스템은 하드웨어 타이머를 이용하여 모터 STEP 펄스를 생성하므로, 메인 루프에서 BNO055를 읽거나 시리얼 명령을 처리하는 동안에도 일정한 주기로 모터를 구동할 수 있다.

---

## 1. 주요 기능

* TMC2209 기반 스테퍼 모터 제어
* 하드웨어 타이머 기반 STEP 펄스 생성
* STEP 주기 **450 µs**
* HOME / TOP 리미트 스위치 안전 제어
* 전원 인가 후 **자동 HOME 동작**
* HOME 위치를 STEP `0`으로 설정
* `H`, `M`, `T` 3단계 명령 제어
* `M` 위치 = **30,000 STEP**
* `T` 위치 = **75,000 STEP**
* BNO055 Yaw 측정
* BNO055 Z축 각속도 측정
* IMU 데이터 **100 ms(10 Hz)** 주기 전송
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

현재 STEP 설정은:

```text
STEP_DELAY_US = 450 µs
```

이며 타이머 인터럽트는 그 절반인:

```text
225 µs
```

마다 발생한다.

따라서 STEP 핀은 다음과 같이 동작한다.

```text
225 µs HIGH
      ↓
225 µs LOW
      ↓
1 STEP 완료
```

즉,

```text
225 µs + 225 µs = 450 µs / STEP
```

이다. 

하드웨어 타이머는 1 MHz, 즉 **1 tick = 1 µs** 기준으로 설정되어 있다. 

---

# 4. 위치 제어

고정 위치는 두 단계로 설정되어 있다.

```cpp
MID_STEP = 30000;
TOP_STEP = 75000;
```

따라서 명령은 다음과 같이 동작한다. 

| 명령  | 동작               |
| --- | ---------------- |
| `H` | HOME 위치로 이동      |
| `M` | 30,000 STEP으로 이동 |
| `T` | 75,000 STEP으로 이동 |

숫자를 직접 입력하는 방식은 사용하지 않으며, 프로그램은 `H`, `M`, `T` 세 명령만 처리한다. 

---

# 5. 전원 인가 및 자동 Homing

전원을 켜면 BNO055 초기화와 하드웨어 타이머 설정 이후 **자동으로 HOME 동작을 시작한다.** 

### 이미 HOME에 있는 경우

```text
전원 ON
 ↓
HOME 스위치 확인
 ↓
눌림
 ↓
currentStep = 0
 ↓
HOME CONFIRMED
```

실제 HOME 스위치가 눌려 있으면 모터를 움직이지 않고 즉시 STEP 0으로 확정한다. 

### 중간 위치에 있는 경우

```text
전원 ON
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

# 6. HOME 스위치 안전 제어

HOME 방향으로 이동하는 동안 HOME 스위치가 눌리면 하드웨어 타이머 ISR에서 즉시 모터를 정지시킨다.

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

따라서 HOME 스위치가 실제 원점 역할을 한다. 

---

# 7. TOP 스위치 안전 제어

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

따라서 정상적인 목표 STEP보다 실제 TOP 리미트 스위치가 먼저 작동하더라도 리프트가 계속 상승하지 않도록 안전장치가 구성되어 있다.

---

# 8. HOME 확인 후 M/T 제어

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
이동 차단
```

HOME이 확인된 이후에만:

```text
M → 30000 STEP
T → 75000 STEP
```

으로 이동한다. 

---

# 9. BNO055 IMU 처리

BNO055는 I2C를 통해 데이터를 읽는다.

측정 데이터:

1. Yaw
2. Z축 각속도(Gyroscope Z)

BNO055의 Yaw는 다음과 같이 변환한다.

```cpp
float raw_yaw = event.orientation.x;

float ros_yaw = 360.0 - raw_yaw;
```

필요한 경우 0~360° 범위로 보정한다. 

---

# 10. IMU 전송 형식

IMU 데이터는 100 ms마다 전송된다.

```text
100 ms = 10 Hz
```

출력 형식은:

```text
<yaw,gyro_z>
```

예:

```text
<125.34,-0.0234>
```

코드에서는 다음과 같이 출력한다.

```text
<
Yaw
,
Gyro Z
>
```



따라서 젯슨이나 다른 상위 시스템에서 다음과 같은 형태로 파싱할 수 있다.

```text
<125.34,-0.0234>
```

---

# 11. 메인 루프와 타이머 인터럽트 역할 분리

이 프로그램은 두 가지 작업을 분리한다.

### 하드웨어 타이머 ISR

모터의 STEP 펄스를 담당한다.

```text
Timer
 ↓
225 µs
 ↓
STEP 토글
 ↓
225 µs
 ↓
STEP 토글
```

모터 제어는 `onTimer()`에서 처리된다. 

### `loop()`

메인 루프에서는:

* `H/M/T` 명령 수신
* HOME 상태 확인
* TOP 상태 확인
* BNO055 데이터 처리

를 담당한다. 

따라서 BNO055를 읽는 작업이 발생하더라도 모터 STEP 생성은 하드웨어 타이머에 의해 독립적으로 수행된다.

---

# 12. 전체 동작 흐름

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
              자동 HOME 시작
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
```

---

# 13. 명령어 요약

| 입력  | 기능      |          목표 |
| --- | ------- | ----------: |
| `H` | HOME 이동 |    HOME 스위치 |
| `M` | 중간 위치   | 30,000 STEP |
| `T` | 상단 위치   | 75,000 STEP |
| 그 외 | 무효 명령   |           - |

`H` 명령은 시스템이 이미 HOME 상태가 아니면 HOME 방향으로 이동하며, HOME 스위치가 눌리면 STEP 0으로 재설정한다. 

---

# 14. 주요 파라미터

```cpp
// 핀
EN_PIN    = 13
STEP_PIN  = 18
DIR_PIN   = 12

HOME_PIN  = 4
TOP_PIN   = 5

// 모터
STEP_DELAY_US = 450

// 위치
MID_STEP = 30000
TOP_STEP = 75000

// IMU
IMU_INTERVAL = 100 ms
```

현재 코드의 핵심 설정값이다.  

---

# 15. 시스템 특징 요약

### 모터

* TMC2209 사용
* EN LOW 활성화
* 하드웨어 타이머 기반 STEP 제어
* 450 µs/STEP
* 225 µs HIGH / 225 µs LOW

### 위치

* HOME = 실제 HOME 리미트 스위치
* M = 30,000 STEP
* T = 75,000 STEP

### 안전

* HOME 리미트 감지 시 즉시 정지
* TOP 리미트 감지 시 즉시 정지
* HOME 확인 전 M/T 명령 차단

### IMU

* BNO055
* I2C `0x28`
* Yaw + Gyro Z
* 10 Hz 전송

### 초기화

```text
전원 ON
→ BNO055 초기화
→ 타이머 초기화
→ 자동 HOME
→ HOME 확인
→ M/T 사용 가능
```

**핵심적으로 이 프로그램은 `HOME 리미트 스위치를 실제 원점으로 사용하고, 하드웨어 타이머로 450 µs 주기의 STEP 펄스를 생성하면서 BNO055 데이터를 동시에 처리하는 ESP32 통합 리프트 제어 코드**라고 보면 돼.
