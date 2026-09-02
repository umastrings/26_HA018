#include <Wire.h>
#include "PS2X_lib.h"
#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"

// ===== QGPMaker 엔코더 초기화 =====
QGPMaker_Encoder encoder1(1);   // Motor 1
QGPMaker_Encoder encoder2(2);   // Motor 2
QGPMaker_Encoder encoder3(3);   // Motor 3
QGPMaker_Encoder encoder4(4);   // Motor 4

bool autoPrint = false;
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500;

// 버튼 상태 추적용 변수
bool circlePressed = false;
bool startPressed = false;
bool selectPressed = false;
bool trianglePressed = false;

// 젯슨 자율주행 모드 플래그
bool jetsonMode = false;

QGPMaker_MotorShield AFMS = QGPMaker_MotorShield(0x60);
PS2X ps2x;

long ARM_MIN[] = {10, 10, 40, 10};
long ARM_MAX[] = {170, 140, 170, 102};

QGPMaker_Servo *Servo1 = AFMS.getServo(0);
QGPMaker_Servo *Servo2 = AFMS.getServo(1);
QGPMaker_Servo *Servo3 = AFMS.getServo(2);
QGPMaker_Servo *Servo4 = AFMS.getServo(3);

QGPMaker_DCMotor *DCMotor_1 = AFMS.getMotor(1);
QGPMaker_DCMotor *DCMotor_2 = AFMS.getMotor(2);
QGPMaker_DCMotor *DCMotor_3 = AFMS.getMotor(3);
QGPMaker_DCMotor *DCMotor_4 = AFMS.getMotor(4);

void printEncoderValues() {
  Serial.print("<"); 
  Serial.print(encoder1.read());
  Serial.print(","); 
  Serial.print(encoder2.read());
  Serial.print(","); 
  Serial.print(encoder3.read());
  Serial.print(","); 
  Serial.print(encoder4.read());
  Serial.println(">");
}

void resetAllEncoders() {
  
  encoder1.write(0);
  encoder2.write(0);
  encoder3.write(0);
  encoder4.write(0);
  Serial.println("[INFO] All encoder values reset.");
}

// ===== 개별 모터 제어 함수 (젯슨 명령용) =====
void applyMotorCommand(QGPMaker_DCMotor *motor, int speed) {
  if (speed > 0) {
    motor->setSpeed(abs(speed));
    motor->run(FORWARD);
  } else if (speed < 0) {
    motor->setSpeed(abs(speed));
    motor->run(BACKWARD);
  } else {
    motor->setSpeed(0);
    motor->run(RELEASE);
  }
}

// ===== 젯슨 통신 파싱 함수 =====
void processJetsonCommand() {
  // 시리얼 버퍼에 데이터가 있을 때만 처리 (젯슨에서 값이 들어올 때마다 실시간 갱신)
  while (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim(); // 공백 및 줄바꿈 제거

    // 포맷 확인: <M,w1,w2,w3,w4>
    if (data.startsWith("<M,") && data.endsWith(">")) {
      data = data.substring(3, data.length() - 1); // "w1,w2,w3,w4" 추출

      int w[4];
      for (int i = 0; i < 3; i++) {
        int commaIndex = data.indexOf(',');
        if (commaIndex != -1) {
          w[i] = data.substring(0, commaIndex).toInt();
          data = data.substring(commaIndex + 1);
        }
      }
      w[3] = data.toInt(); // 마지막 값

      // 파싱된 속도를 모터에 적용
      applyMotorCommand(DCMotor_1, w[0]);
      applyMotorCommand(DCMotor_2, w[1]);
      applyMotorCommand(DCMotor_3, w[2]);
      applyMotorCommand(DCMotor_4, w[3]);
    }
  }
}

void forward() {
  DCMotor_1->setSpeed(80); DCMotor_1->run(FORWARD);
  DCMotor_2->setSpeed(80); DCMotor_2->run(FORWARD);
  DCMotor_3->setSpeed(80); DCMotor_3->run(FORWARD);
  DCMotor_4->setSpeed(80); DCMotor_4->run(FORWARD);
}

void turnLeft() {
  DCMotor_1->setSpeed(100); DCMotor_1->run(BACKWARD);
  DCMotor_2->setSpeed(100); DCMotor_2->run(BACKWARD);
  DCMotor_3->setSpeed(90); DCMotor_3->run(FORWARD);
  DCMotor_4->setSpeed(90); DCMotor_4->run(FORWARD);
}

void turnRight() {
  DCMotor_1->setSpeed(80); DCMotor_1->run(FORWARD);
  DCMotor_2->setSpeed(80); DCMotor_2->run(FORWARD);
  DCMotor_3->setSpeed(100); DCMotor_3->run(BACKWARD);
  DCMotor_4->setSpeed(100); DCMotor_4->run(BACKWARD);
}

void moveLeft() {
  DCMotor_1->setSpeed(100); DCMotor_1->run(BACKWARD);
  DCMotor_2->setSpeed(100); DCMotor_2->run(FORWARD);
  DCMotor_3->setSpeed(100); DCMotor_3->run(BACKWARD);
  DCMotor_4->setSpeed(100); DCMotor_4->run(FORWARD);
}

void moveRight() {
  DCMotor_1->setSpeed(100); DCMotor_1->run(FORWARD);
  DCMotor_2->setSpeed(100); DCMotor_2->run(BACKWARD);
  DCMotor_3->setSpeed(100); DCMotor_3->run(FORWARD);
  DCMotor_4->setSpeed(100); DCMotor_4->run(BACKWARD);
}

void backward() {
  DCMotor_1->setSpeed(100); DCMotor_1->run(BACKWARD);
  DCMotor_2->setSpeed(100); DCMotor_2->run(BACKWARD);
  DCMotor_3->setSpeed(100); DCMotor_3->run(BACKWARD);
  DCMotor_4->setSpeed(100); DCMotor_4->run(BACKWARD);
}

void stopMoving() {
  DCMotor_1->setSpeed(0); DCMotor_1->run(RELEASE);
  DCMotor_2->setSpeed(0); DCMotor_2->run(RELEASE);
  DCMotor_3->setSpeed(0); DCMotor_3->run(RELEASE);
  DCMotor_4->setSpeed(0); DCMotor_4->run(RELEASE);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); 

  Serial.println("START(PSB_START): Print encoder values once");
  Serial.println("SELECT(PSB_SELECT): Reset encoder values to 0");
  Serial.println("CIRCLE(PSB_CIRCLE): Toggle auto print");
  Serial.println("TRIANGLE(PSB_TRIANGLE): Toggle Jetson Auto Mode");

  AFMS.begin(100);
  int error = 0;

  do {
    error = ps2x.config_gamepad(13, 11, 10, 12, true, true);
    if (error == 0) {
      break;
    } else {
      delay(100);
    }
  } while (1);
  
  Servo1->writeServo(90);
  Servo2->writeServo(90);
  Servo3->writeServo(90);
  Servo4->writeServo(60);
  Serial.println("=== Setup Complete ===");
}

void loop() {
  ps2x.read_gamepad(false, 0);
  delay(30);

  // CIRCLE: autoPrint 토글
  boolean currentCircle = ps2x.Button(PSB_CIRCLE);
  if (currentCircle && !circlePressed) {
    autoPrint = !autoPrint;
    Serial.print("Auto print: ");
    Serial.println(autoPrint ? "ON" : "OFF");
  }
  circlePressed = currentCircle;
  
  // START: 엔코더 값 출력
  boolean currentStart = ps2x.Button(PSB_START);
  if (currentStart && !startPressed) {
    printEncoderValues();
  }
  startPressed = currentStart;
  
  // SELECT: 엔코더 리셋
  boolean currentSelect = ps2x.Button(PSB_SELECT);
  if (currentSelect && !selectPressed) {
    resetAllEncoders();
  }
  selectPressed = currentSelect;

  // ===== TRIANGLE: 젯슨 모드 토글 (한번 누르면 켜짐, 다시 누르면 꺼짐) =====
  boolean currentTriangle = ps2x.Button(PSB_TRIANGLE);
  if (currentTriangle && !trianglePressed) {
    jetsonMode = !jetsonMode; // 상태 반전 (토글)
    
    Serial.print("[INFO] Jetson Auto Mode: ");
    Serial.println(jetsonMode ? "ON" : "OFF");
    
    if (!jetsonMode) {
      stopMoving(); // 젯슨 모드를 끄고 수동 조작으로 돌아올 때 안전을 위해 모터 정지
    }
  }
  trianglePressed = currentTriangle;


  // ===== 모터 제어 =====
  
  if (jetsonMode) {
    // 1. 젯슨 자동 제어 모드가 ON인 경우
    // 계속해서 processJetsonCommand 함수를 실행하며 젯슨의 명령을 받아 갱신합니다.
    processJetsonCommand();
  } else {
    // 2. 젯슨 자동 제어 모드가 OFF인 경우 (기존 매뉴얼 조이스틱 조작)
    if (ps2x.Button(PSB_PAD_UP)) {
      turnLeft();
    } else if (ps2x.Button(PSB_PAD_DOWN)) {
      turnRight();
    } else if (ps2x.Button(PSB_PAD_LEFT)) {
       if (ps2x.Button(PSB_L2)) {
        DCMotor_2->setSpeed(200); DCMotor_2->run(BACKWARD);
        DCMotor_4->setSpeed(200); DCMotor_4->run(BACKWARD);
      } else if (ps2x.Button(PSB_R2)) {
        DCMotor_1->setSpeed(200); DCMotor_1->run(BACKWARD);
        DCMotor_3->setSpeed(200); DCMotor_3->run(BACKWARD);
      } else {
        backward();
      }
    } else if (ps2x.Button(PSB_PAD_RIGHT)) {
      if (ps2x.Button(PSB_L2)) {
        DCMotor_2->setSpeed(200); DCMotor_2->run(FORWARD);
        DCMotor_4->setSpeed(200); DCMotor_4->run(FORWARD);
      } else if (ps2x.Button(PSB_R2)) {
        DCMotor_1->setSpeed(200); DCMotor_1->run(FORWARD);
        DCMotor_3->setSpeed(200); DCMotor_3->run(FORWARD);
      } else {
        forward();
      }
    } else if (ps2x.Button(PSB_L1)) {
      moveLeft();
    } else if (ps2x.Button(PSB_R1)) {
      moveRight();
    } else {
      stopMoving(); // 조이스틱 아무 버튼도 안 누르면 모터 정지
    }
  }

  // ===== 서보 제어 (Servo 1 ~ 4) =====
  if (ps2x.Analog(PSS_LX) > 240) {
    if (Servo1->readDegrees() > ARM_MIN[0]) Servo1->writeServo((Servo1->readDegrees() - 1));
  } else if (ps2x.Analog(PSS_LX) < 10) {
    if (Servo1->readDegrees() < ARM_MAX[0]) Servo1->writeServo((Servo1->readDegrees() + 1));
  }

  if (ps2x.Analog(PSS_LY) > 240) {
    if (Servo2->readDegrees() > ARM_MIN[1]) Servo2->writeServo((Servo2->readDegrees() - 1));
  } else if (ps2x.Analog(PSS_LY) < 10) {
    if (Servo2->readDegrees() < ARM_MAX[1]) Servo2->writeServo((Servo2->readDegrees() + 1));
  }

  if (ps2x.Analog(PSS_RY) > 240) {
    if (Servo3->readDegrees() > ARM_MIN[2]) Servo3->writeServo((Servo3->readDegrees() - 1));
  } else if (ps2x.Analog(PSS_RY) < 10) {
    if (Servo3->readDegrees() < ARM_MAX[2]) Servo3->writeServo((Servo3->readDegrees() + 1));
  }

  if (ps2x.Analog(PSS_RX) > 240) {
    if (Servo4->readDegrees() > ARM_MIN[3]) Servo4->writeServo((Servo4->readDegrees() - 1));
  } else if (ps2x.Analog(PSS_RX) < 10) {
    if (Servo4->readDegrees() < ARM_MAX[3]) Servo4->writeServo((Servo4->readDegrees() + 1));
  }

  // 자동 출력
  if (autoPrint && millis() - lastPrintTime >= printInterval) {
    lastPrintTime = millis();
    printEncoderValues();
  }

  delay(2);
}