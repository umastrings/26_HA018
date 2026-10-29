#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// =====================================================
// BNO055
// =====================================================
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

Adafruit_BNO055 bno(55, 0x28, &Wire);

// =====================================================
// MOTOR PIN
// =====================================================
#define EN_PIN    13
#define STEP_PIN  18
#define DIR_PIN   12

// =====================================================
// LIMIT SWITCH
// NO 타입
// 평상시 HIGH
// 눌리면 GND -> LOW
// =====================================================
#define HOME_PIN  4
#define TOP_PIN   5

// =====================================================
// MOTOR SPEED
//
// 1 STEP = 450 us
//
// HIGH 225 us
// LOW  225 us
// = 450 us / STEP
// =====================================================
#define STEP_DELAY_US       450
#define TIMER_INTERVAL_US   (STEP_DELAY_US / 2)

// =====================================================
// DIRECTION
//
// 실제 기계가 반대로 움직이면 HIGH / LOW를 서로 변경
// =====================================================
#define DIR_UP    LOW
#define DIR_DOWN  HIGH

// =====================================================
// FIXED POSITION
// =====================================================
const long MID_STEP = 30000;
const long TOP_STEP = 75000;

// =====================================================
// MOTOR STATE
// =====================================================
volatile long currentStep = 0;
volatile long targetStep = 0;

volatile bool running = false;
volatile bool homing = false;

volatile int stepDirection = 1;

volatile bool stepState = LOW;

// =====================================================
// HOME CONFIRMED
//
// 전원 ON 직후 false
// HOME 스위치 확인 후 true
// =====================================================
volatile bool homeConfirmed = false;

// =====================================================
// TIMER
// =====================================================
hw_timer_t *timer = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// IMU
// =====================================================
unsigned long lastImuTime = 0;

// ★ 10/4 — 10Hz -> 50Hz. 패킷 ~30B x 50Hz = 1.5KB/s, 115200(11.5KB/s)에 충분.
//   EKF 는 gyro_z(vyaw)를 적분하므로 자주 받을수록 회전 추정이 매끄럽다.
const unsigned long IMU_INTERVAL = 20;

// =====================================================
// LIMIT SWITCH
// =====================================================
bool homePressed()
{
  return digitalRead(HOME_PIN) == LOW;
}

bool topPressed()
{
  return digitalRead(TOP_PIN) == LOW;
}

// =====================================================
// TIMER SPEED
// =====================================================
void setNormalSpeed()
{
  timerAlarm(
    timer,
    TIMER_INTERVAL_US,
    true,
    0
  );
}

// =====================================================
// MOTOR STOP
// =====================================================
void stopMotor()
{
  portENTER_CRITICAL(&timerMux);

  running = false;
  homing = false;

  targetStep = currentStep;

  stepState = LOW;

  digitalWrite(STEP_PIN, LOW);

  portEXIT_CRITICAL(&timerMux);

  Serial.print("STOP | STEP = ");
  Serial.println(currentStep);
}

// =====================================================
// HARDWARE TIMER ISR
//
// 225us마다 STEP 핀 토글
//
// HIGH 225us
// LOW  225us
//
// 실제 1 STEP = 450us
// =====================================================
void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);

  if (running)
  {
    // =================================================
    // HOME 방향 이동 중
    // =================================================
    if (stepDirection == -1)
    {
      // HOME 스위치 눌림
      if (digitalRead(HOME_PIN) == LOW)
      {
        running = false;
        homing = false;

        currentStep = 0;
        targetStep = 0;

        stepState = LOW;

        digitalWrite(STEP_PIN, LOW);

        homeConfirmed = true;

        portEXIT_CRITICAL_ISR(&timerMux);

        return;
      }
    }

    // =================================================
    // 상승 중 TOP 확인
    // =================================================
    if (stepDirection == 1)
    {
      // TOP 스위치 눌림
      if (digitalRead(TOP_PIN) == LOW)
      {
        running = false;
        homing = false;

        currentStep = TOP_STEP;
        targetStep = TOP_STEP;

        stepState = LOW;

        digitalWrite(STEP_PIN, LOW);

        portEXIT_CRITICAL_ISR(&timerMux);

        return;
      }
    }

    // =================================================
    // STEP PULSE
    // =================================================
    stepState = !stepState;

    digitalWrite(STEP_PIN, stepState);

    // =================================================
    // LOW가 되는 순간
    // 실제 STEP 1회 완료
    // =================================================
    if (stepState == LOW)
    {
      // =================================================
      // ★ 중요 ★
      //
      // Homing 중에는 currentStep을 변경하지 않는다.
      //
      // 전원 ON 직후에는 실제 위치를 모르기 때문.
      //
      // HOME 스위치가 눌리는 순간에만
      // currentStep = 0으로 확정한다.
      // =================================================
      if (!homing)
      {
        currentStep += stepDirection;

        // =================================================
        // 일반 이동 목표 도착
        // =================================================
        if (currentStep == targetStep)
        {
          running = false;
        }

        // =================================================
        // 안전 범위
        // =================================================
        if (currentStep < 0)
        {
          currentStep = 0;
          running = false;
        }

        if (currentStep > TOP_STEP)
        {
          currentStep = TOP_STEP;
          running = false;
        }
      }

      // =================================================
      // 정지 시 STEP LOW
      // =================================================
      if (!running)
      {
        stepState = LOW;
        digitalWrite(STEP_PIN, LOW);
      }
    }
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}

// =====================================================
// MOVE TO FIXED POSITION
//
// M = 30000
// T = 75000
// =====================================================
void moveToStep(long target)
{
  // ===================================================
  // HOME 확인 전에는 이동 금지
  // ===================================================
  if (!homeConfirmed)
  {
    Serial.println("HOME NOT CONFIRMED");
    Serial.println("WAIT FOR AUTO HOMING");
    return;
  }

  // ===================================================
  // 목표값 제한
  // ===================================================
  if (target < 0)
    target = 0;

  if (target > TOP_STEP)
    target = TOP_STEP;

  Serial.println();
  Serial.print("TARGET STEP = ");
  Serial.println(target);

  // ===================================================
  // 이미 목표 위치
  // ===================================================
  if (target == currentStep)
  {
    Serial.println("ALREADY AT TARGET");
    return;
  }

  // ===================================================
  // 상승
  // ===================================================
  if (target > currentStep)
  {
    // TOP 스위치가 이미 눌려 있으면 상승 금지
    if (topPressed())
    {
      Serial.println("TOP LIMIT -> UP DISABLED");
      return;
    }

    portENTER_CRITICAL(&timerMux);

    targetStep = target;

    stepDirection = 1;

    homing = false;

    digitalWrite(DIR_PIN, DIR_UP);

    running = true;

    portEXIT_CRITICAL(&timerMux);

    setNormalSpeed();

    Serial.print("UP : ");
    Serial.print(target - currentStep);
    Serial.println(" steps");
  }

  // ===================================================
  // 하강
  // ===================================================
  else
  {
    // HOME 스위치가 이미 눌려 있으면
    if (homePressed())
    {
      portENTER_CRITICAL(&timerMux);

      currentStep = 0;
      targetStep = 0;

      running = false;
      homing = false;

      stepState = LOW;

      digitalWrite(STEP_PIN, LOW);

      homeConfirmed = true;

      portEXIT_CRITICAL(&timerMux);

      Serial.println("HOME LIMIT -> STEP = 0");

      return;
    }

    portENTER_CRITICAL(&timerMux);

    targetStep = target;

    stepDirection = -1;

    homing = false;

    digitalWrite(DIR_PIN, DIR_DOWN);

    running = true;

    portEXIT_CRITICAL(&timerMux);

    setNormalSpeed();

    Serial.print("DOWN : ");
    Serial.print(currentStep - target);
    Serial.println(" steps");
  }
}

// =====================================================
// HOME
//
// H 명령 또는 전원 ON 자동 Homing
// =====================================================
void goHome(bool automatic = false)
{
  if (automatic)
  {
    Serial.println();
    Serial.println("=== AUTO HOMING ===");
  }
  else
  {
    Serial.println();
    Serial.println("=== GO HOME ===");
  }

  // ===================================================
  // 이미 HOME이면 바로 0점 확정
  // ===================================================
  if (homePressed())
  {
    portENTER_CRITICAL(&timerMux);

    running = false;
    homing = false;

    currentStep = 0;
    targetStep = 0;

    stepState = LOW;

    digitalWrite(STEP_PIN, LOW);

    homeConfirmed = true;

    portEXIT_CRITICAL(&timerMux);

    Serial.println("HOME LIMIT DETECTED");
    Serial.println("HOME POSITION = STEP 0");
    Serial.println("HOME CONFIRMED");

    return;
  }

  // ===================================================
  // HOME 방향으로 이동
  // ===================================================
  portENTER_CRITICAL(&timerMux);

  stepDirection = -1;

  homing = true;

  targetStep = 0;

  digitalWrite(DIR_PIN, DIR_DOWN);

  running = true;

  homeConfirmed = false;

  portEXIT_CRITICAL(&timerMux);

  // 일반 STEP 주기 450us
  setNormalSpeed();

  if (automatic)
  {
    Serial.println("AUTO HOMING: MOVING DOWN...");
  }
  else
  {
    Serial.println("MOVING DOWN TO HOME...");
  }
}

// =====================================================
// LIFT STATE (H/M/T/-) — <I,...,state> 필드용
//
// 이동 중이거나 세 고정 위치가 아니면 '-'.
// (★ 9/5 추가 — <D,state,checksum> 완료 이벤트는 아직 미구현.
//  이 coarse state만으로도 /lift/state 자가복구 경로가 동작함.
//  체크섬 이벤트는 여유 생기면 별도로 추가할 것)
// =====================================================
char currentStateChar()
{
  if (running)
  {
    return '-';
  }

  if (homeConfirmed && currentStep == 0)
  {
    return 'H';
  }

  if (currentStep == MID_STEP)
  {
    return 'M';
  }

  if (currentStep == TOP_STEP)
  {
    return 'T';
  }

  return '-';
}

// =====================================================
// BNO055
// =====================================================
void readAndSendIMU()
{
  sensors_event_t event;

  bno.getEvent(&event);

  imu::Vector<3> gyro =
      bno.getVector(
        Adafruit_BNO055::VECTOR_GYROSCOPE
      );

  float raw_yaw = event.orientation.x;

  float ros_yaw = 360.0 - raw_yaw;

  if (ros_yaw >= 360.0)
  {
    ros_yaw = 0.0;
  }

  if (ros_yaw < 0.0)
  {
    ros_yaw += 360.0;
  }

  // =================================================
  // ★ 9/5 — ROS/EKF는 라디안 기대. BNO055 Euler(도)뿐 아니라
  //   getVector(VECTOR_GYROSCOPE)도 deg/s(dps) 단위라 같이 변환.
  //   gyro 단위는 90도 회전 테스트로 실측 재확인할 것.
  // =================================================
  float yaw_rad = ros_yaw * DEG_TO_RAD;
  float gyro_z_rad = gyro.z() * DEG_TO_RAD;

  // =================================================
  // lift_mm: 스텝->mm 환산 상수 아직 미실측 (TODO).
  //   확정되면 currentStep * MM_PER_STEP 로 교체.
  //   지금은 use_virtual_lift:=true로 운용해 이 값 자체는 안 쓰임 —
  //   프로토콜 필드 개수만 맞추기 위한 자리채움.
  // =================================================
  float lift_mm_placeholder = 0.0;

  char state = currentStateChar();

  // =================================================
  // <I,yaw_rad,gyro_z_rad,lift_mm,state>
  // =================================================
  Serial.print("<I,");
  Serial.print(yaw_rad, 4);
  Serial.print(",");
  Serial.print(gyro_z_rad, 4);
  Serial.print(",");
  Serial.print(lift_mm_placeholder, 1);
  Serial.print(",");
  Serial.print(state);
  Serial.println(">");
}

// =====================================================
// COMMAND
//
// H = HOME
// M = 30000
// T = 75000
// =====================================================
void processCommand(String input)
{
  input.trim();

  if (input.length() == 0)
    return;

  // ===================================================
  // H = HOME
  // ===================================================
  if (input.equalsIgnoreCase("H"))
  {
    if (running)
    {
      Serial.println("MOTOR RUNNING");
      return;
    }

    goHome(false);

    return;
  }

  // ===================================================
  // M = 30000
  // ===================================================
  if (input.equalsIgnoreCase("M"))
  {
    if (!homeConfirmed)
    {
      Serial.println("HOME NOT CONFIRMED");
      Serial.println("AUTO HOMING IN PROGRESS...");
      return;
    }

    if (running)
    {
      Serial.println("MOTOR RUNNING");
      return;
    }

    Serial.println("MOVE TO MIDDLE");

    moveToStep(MID_STEP);

    return;
  }

  // ===================================================
  // T = 75000
  // ===================================================
  if (input.equalsIgnoreCase("T"))
  {
    if (!homeConfirmed)
    {
      Serial.println("HOME NOT CONFIRMED");
      Serial.println("AUTO HOMING IN PROGRESS...");
      return;
    }

    if (running)
    {
      Serial.println("MOTOR RUNNING");
      return;
    }

    Serial.println("MOVE TO TOP");

    moveToStep(TOP_STEP);

    return;
  }

  // ===================================================
  // 잘못된 명령
  // ===================================================
  Serial.println("INVALID COMMAND");
  Serial.println("USE ONLY: H / M / T");
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  // ===================================================
  // MOTOR
  // ===================================================
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);

  // ===================================================
  // LIMIT SWITCH
  // ===================================================
  pinMode(HOME_PIN, INPUT_PULLUP);
  pinMode(TOP_PIN, INPUT_PULLUP);

  // ===================================================
  // BNO055
  // ===================================================
  Wire.begin(
    I2C_SDA_PIN,
    I2C_SCL_PIN
  );

  Serial.println();
  Serial.println("BNO055 START...");

  // ★ 10/4 — NDOF(기본값) -> IMUPLUS.
  //   NDOF 는 지자기로 yaw 를 보정하는데, 메카넘 모터 4개 + 배터리 배선 옆이라
  //   주행 중 yaw 가 튄다. 또 yaw 가 자북 기준이라 odom 시작 방향이 틀어졌다.
  //   IMUPLUS = 자이로+가속도 6축 퓨전, 전원 ON 시 정면이 yaw 0.
  //   (장기 yaw 드리프트는 SLAM/AMCL 이 라이다로 보정)
  if (!bno.begin(OPERATION_MODE_IMUPLUS))
  {
    // 브리지는 <I> 가 안 오면 'IMU 수신 끊김' ERROR 를 낸다.
    // raw 시리얼로 봤을 때 원인이 바로 보이도록 1초마다 반복 출력.
    while (1)
    {
      Serial.println("BNO055_ERROR");
      delay(1000);
    }
  }

  bno.setExtCrystalUse(true);

  delay(1000);

  Serial.println("BNO055 OK (IMUPLUS)");

  // ★ 전원 ON 직후 자이로 바이어스 자동 보정 구간 — 이 동안 로봇을 움직이지 말 것.
  //   (gyro calib 상태 0~3, 3 이면 완료. 최대 3초만 기다리고 진행)
  {
    uint8_t sys, g, a, m;
    unsigned long t0 = millis();
    do
    {
      bno.getCalibration(&sys, &g, &a, &m);
      if (g == 3) break;
      delay(100);
    } while (millis() - t0 < 3000);
    Serial.print("BNO055 GYRO CALIB = ");
    Serial.println(g);
  }

  // ===================================================
  // HARDWARE TIMER
  //
  // 1MHz = 1us/tick
  // 225us마다 토글
  // 450us마다 1 STEP
  // ===================================================
  timer = timerBegin(1000000);

  timerAttachInterrupt(
    timer,
    &onTimer
  );

  timerAlarm(
    timer,
    TIMER_INTERVAL_US,
    true,
    0
  );

  Serial.println("TIMER OK");

  // ===================================================
  // SYSTEM START
  // ===================================================
  Serial.println();
  Serial.println("==============================");
  Serial.println("       SYSTEM START");
  Serial.println("==============================");

  Serial.println("AUTO HOMING...");

  // ===================================================
  // ★ 전원 ON 자동 HOME
  // ===================================================
  goHome(true);
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  // ===================================================
  // 1. COMMAND
  // ===================================================
  if (Serial.available())
  {
    String input =
      Serial.readStringUntil('\n');

    processCommand(input);
  }

  // ===================================================
  // 2. HOME LIMIT
  //
  // ISR에서도 확인하지만
  // 메인 루프에서도 이중 확인
  // ===================================================
  if (homePressed())
  {
    if (running && stepDirection == -1)
    {
      portENTER_CRITICAL(&timerMux);

      running = false;
      homing = false;

      currentStep = 0;
      targetStep = 0;

      stepState = LOW;

      digitalWrite(STEP_PIN, LOW);

      homeConfirmed = true;

      portEXIT_CRITICAL(&timerMux);

      Serial.println();
      Serial.println("================================");
      Serial.println("HOME LIMIT DETECTED");
      Serial.println("HOME POSITION = STEP 0");
      Serial.println("HOME CONFIRMED");
      Serial.println("M / T ENABLED");
      Serial.println("================================");
    }
  }

  // ===================================================
  // 3. TOP LIMIT
  //
  // 상승 중 TOP 스위치 눌리면 즉시 정지
  // ===================================================
  if (topPressed())
  {
    if (running && stepDirection == 1)
    {
      portENTER_CRITICAL(&timerMux);

      running = false;
      homing = false;

      currentStep = TOP_STEP;
      targetStep = TOP_STEP;

      stepState = LOW;

      digitalWrite(STEP_PIN, LOW);

      portEXIT_CRITICAL(&timerMux);

      Serial.println();
      Serial.println("================================");
      Serial.println("TOP LIMIT DETECTED");
      Serial.println("STEP = 75000");
      Serial.println("MOTOR STOPPED");
      Serial.println("================================");
    }
  }

  // ===================================================
  // 4. BNO055
  //
  // 100ms = 10Hz
  // ===================================================
  unsigned long currentTime = millis();

  if (currentTime - lastImuTime >= IMU_INTERVAL)
  {
    lastImuTime = currentTime;

    readAndSendIMU();
  }
}