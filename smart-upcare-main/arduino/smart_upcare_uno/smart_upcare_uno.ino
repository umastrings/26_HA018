/*
 * smart_upcare_uno.ino — Smart Up-Care UNO 펌웨어 (실차 검증본)
 *
 * ★ 핸드오프 문서 §2 프로토콜에 맞춤 (마커 방식)
 *   업링크 (UNO → 젯슨)  25Hz :  <E,c1,c2,c3,c4>\n     누적 엔코더 카운트
 *   다운링크 (젯슨 → UNO) 50Hz :  <M,pwm1,pwm2,pwm3,pwm4>\n   PWM -255~255
 *
 * 휠 순서: c1/pwm1=M1(FL) c2/pwm2=M2(RL) c3/pwm3=M3(RR) c4/pwm4=M4(FR)
 *
 * ── 핸드오프 §6 "확인 필요" 항목 회신 ──────────────────────────
 *  ① Setspeed 16× 스케일 : 펌웨어에서 별도 스케일 안 함.
 *     QGPMaker 라이브러리 setSpeed(0~255) 내부에서 PCA9685 12bit(×16) 변환.
 *     → 젯슨이 보내는 최종 PWM(-255~255)을 그대로 인가. 이중 적용 없음. ✅
 *  ② 300ms 워치독 : 아래 CMD_TIMEOUT_MS=300 으로 펌웨어에도 구현됨 (이중 방어). ✅
 *  ③ PID : 없음(개루프). 속도 폐루프는 젯슨(serial_bridge FF+PI)이 담당. ✅
 *
 * ── 실차 확인 사항 ────────────────────────────────────────────
 *  · 포트: CH340 클론 UNO → /dev/ttyCH341USB0 (ttyACM0 아님, 드라이버 별도 빌드)
 *  · 모터 방향 보정은 젯슨 wheel_sign 파라미터에서 흡수 (펌웨어는 raw 인가)
 *  · IMU/리프트는 이 보드에 없음 → ESP32 #1 담당 (<I,...>)
 */
#include <Wire.h>
#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"

const long          BAUD             = 115200;
const int           UPLINK_HZ        = 25;
const unsigned long UPLINK_PERIOD_MS = 1000UL / UPLINK_HZ;
const unsigned long CMD_TIMEOUT_MS   = 300;      // ★ 워치독 (핸드오프 §2-2 ②)

QGPMaker_MotorShield AFMS = QGPMaker_MotorShield(0x60);
QGPMaker_DCMotor *M1, *M2, *M3, *M4;
QGPMaker_Encoder encoder1(1), encoder2(2), encoder3(3), encoder4(4);   // CPR 4320

unsigned long last_cmd_ms = 0, last_uplink_ms = 0;
char    rx_buf[64];
uint8_t rx_len = 0;
bool    in_packet = false;

void applyMotorCommand(QGPMaker_DCMotor *m, int pwm) {
  if (pwm >  255) pwm =  255;
  if (pwm < -255) pwm = -255;
  if (pwm > 0)      { m->setSpeed(pwm);  m->run(FORWARD); }
  else if (pwm < 0) { m->setSpeed(-pwm); m->run(BACKWARD); }
  else              { m->setSpeed(0);    m->run(RELEASE); }
}

void stopMotors() {
  applyMotorCommand(M1,0); applyMotorCommand(M2,0);
  applyMotorCommand(M3,0); applyMotorCommand(M4,0);
}

// 다운링크 <M,pwm1,pwm2,pwm3,pwm4>
void parseDownlink() {
  if (rx_buf[0] != 'M' || rx_buf[1] != ',') return;
  char* p = rx_buf + 2;
  int w[4] = {0,0,0,0};
  for (int i = 0; i < 4; i++) {
    w[i] = atoi(p);
    p = strchr(p, ',');
    if (!p) break;
    p++;
  }
  applyMotorCommand(M1,w[0]); applyMotorCommand(M2,w[1]);
  applyMotorCommand(M3,w[2]); applyMotorCommand(M4,w[3]);
  last_cmd_ms = millis();
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<')                  { in_packet = true; rx_len = 0; }
    else if (c == '>' && in_packet){ rx_buf[rx_len] = '\0'; parseDownlink(); in_packet = false; }
    else if (in_packet) {
      if (rx_len < sizeof(rx_buf) - 1) rx_buf[rx_len++] = c;
      else in_packet = false;
    }
  }
}

// 업링크 <E,c1,c2,c3,c4>
void sendUplink() {
  char out[64];
  snprintf(out, sizeof(out), "<E,%ld,%ld,%ld,%ld>",
           (long)encoder1.read(), (long)encoder2.read(),
           (long)encoder3.read(), (long)encoder4.read());
  Serial.println(out);
}

void setup() {
  Serial.begin(BAUD);
  AFMS.begin(100);
  M1 = AFMS.getMotor(1); M2 = AFMS.getMotor(2);
  M3 = AFMS.getMotor(3); M4 = AFMS.getMotor(4);
  stopMotors();
  last_cmd_ms = millis();
}

void loop() {
  readSerial();
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) stopMotors();   // 워치독
  unsigned long now = millis();
  if (now - last_uplink_ms >= UPLINK_PERIOD_MS) {
    last_uplink_ms = now;
    sendUplink();
  }
}
