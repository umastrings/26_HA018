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
// ACTIVE COMMAND (완료 응답용)
//
// 현재 진행 중인 명령: 'H' / 'M' / 'T' / 0(없음)
// loop()에서 running: true -> false 전환을 감지해
// 이 값을 기준으로 xxx_DONE을 Jetson에 전송한다.
// =====================================================
volatile char activeCmd = 0;
bool prevRunning = false;

// =====================================================
// TIMER
// =====================================================
hw_timer_t *timer = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// IMU
// =====================================================
unsigned long lastImuTime = 0;

const unsigned long IMU_INTERVAL = 100;

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
// COMPLETE RESPONSE (Jetson으로 완료 신호 전송)
// =====================================================
void sendDone(char cmd)
{
  switch (cmd)
  {
    case 'H':
      Serial.println("H_DONE");
      break;

    case 'M':
      Serial.println("M_DONE");
      break;

    case 'T':
      Serial.println("T_DONE");
      break;

    default:
      break;
  }
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
//
// cmd : 'M' 또는 'T' (완료 응답 구분용)
// =====================================================
void moveToStep(long target, char cmd)
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
  // 이미 목표 위치 -> 즉시 완료 처리
  // ===================================================
  if (target == currentStep)
  {
    Serial.println("ALREADY AT TARGET");

    sendDone(cmd);

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

    activeCmd = cmd;

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
    // HOME 스위치가 이미 눌려 있으면 -> 즉시 완료 처리
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

      sendDone(cmd);

      return;
    }

    portENTER_CRITICAL(&timerMux);

    targetStep = target;

    stepDirection = -1;

    homing = false;

    digitalWrite(DIR_PIN, DIR_DOWN);

    running = true;

    portEXIT_CRITICAL(&timerMux);

    activeCmd = cmd;

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
  // 이미 HOME이면 바로 0점 확정 -> 즉시 완료 처리
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

    sendDone('H');

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

  activeCmd = 'H';

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
  // <yaw,gyro_z>
  // =================================================
  Serial.print("<");
  Serial.print(ros_yaw, 2);
  Serial.print(",");
  Serial.print(gyro.z(), 4);
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

    moveToStep(MID_STEP, 'M');

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

    moveToStep(TOP_STEP, 'T');

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

  if (!bno.begin())
  {
    Serial.println("BNO055_ERROR");

    while (1)
    {
      delay(1000);
    }
  }

  bno.setExtCrystalUse(true);

  delay(1000);

  Serial.println("BNO055 OK");

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
  // 4. COMPLETE RESPONSE
  //
  // running: true -> false 로 바뀌는 순간을 감지해서
  // 진행 중이던 명령(activeCmd)에 대한 완료 응답 전송.
  //
  // 정상 도착 / HOME·TOP 리미트 감지 / 안전 범위 클램프 등
  // running이 꺼지는 모든 경로를 이 한 곳에서 통합 처리한다.
  // =================================================== 
  bool nowRunning = running;

  if (prevRunning && !nowRunning)
  {
    if (activeCmd != 0)
    {
      sendDone(activeCmd);
      activeCmd = 0;
    }
  }

  prevRunning = nowRunning;

  // ===================================================
  // 5. BNO055
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
