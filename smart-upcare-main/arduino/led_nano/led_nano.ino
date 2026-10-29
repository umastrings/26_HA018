/*
 * led_nano.ino — Smart Up-Care LED module (Nano mini Type-C, CMODULE-23)
 *
 * Downlink (Jetson -> Nano) 115200: <L,I> <L,P> <L,E> <L,S>
 *   I: green solid    P: blue breathing    E: red blink 2Hz    S: red solid
 *   No command for 3 s: yellow slow blink (link lost)
 * Uplink (Nano -> Jetson):
 *   LED_NANO READY      once at boot
 *   <X,1> / <X,0>       e-stop pressed / released, on change + every 1 s
 *
 * Pins: D6 -> 300~330 ohm -> strip DIN
 *       D3 <- e-stop a-contact (NO) to GND, INPUT_PULLUP, LOW = pressed
 *   CMODULE-23 has no D2 header (D2 drives onboard RGB LED1~3), so e-stop uses D3.
 *
 * Upload: arduino:avr:nano:cpu=atmega328old (old bootloader)
 */
#include <Adafruit_NeoPixel.h>

const uint8_t  STRIP_PIN    = 6;
const uint8_t  ESTOP_PIN    = 3;
const uint16_t NUM_LEDS     = 144;   // 72 + 72 LEDs (counted 10/17)
const uint8_t  BRIGHTNESS   = 25;    // USB power: 144 LEDs yellow ~0.45 A
const long     BAUD         = 115200;
const unsigned long CMD_TIMEOUT_MS = 3000;
const unsigned long DEBOUNCE_MS    = 20;
const unsigned long ESTOP_REPORT_MS = 1000;

Adafruit_NeoPixel strip(NUM_LEDS, STRIP_PIN, NEO_GRB + NEO_KHZ800);

char state = 0;                      // last commanded state, 0 = none yet
unsigned long last_cmd_ms = 0;

bool estop = false;                  // debounced: true = pressed
bool estop_raw_last = false;
unsigned long estop_change_ms = 0, estop_report_ms = 0;

char rx_buf[16];
uint8_t rx_len = 0;
bool in_packet = false;

uint32_t last_color = 1;             // force first show()

void fill(uint32_t c) {
  if (c == last_color) return;
  last_color = c;
  for (uint16_t i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, c);
  strip.show();
}

void reportEstop() {
  Serial.print(estop ? F("<X,1>\n") : F("<X,0>\n"));
  estop_report_ms = millis();
}

void parsePacket() {
  if (rx_len == 3 && rx_buf[0] == 'L' && rx_buf[1] == ',') {
    char s = rx_buf[2];
    if (s == 'I' || s == 'P' || s == 'E' || s == 'S') {
      state = s;
      last_cmd_ms = millis();
    }
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') { in_packet = true; rx_len = 0; }
    else if (c == '>') { if (in_packet) { rx_buf[rx_len] = 0; parsePacket(); } in_packet = false; }
    else if (in_packet) {
      if (rx_len < sizeof(rx_buf) - 1) rx_buf[rx_len++] = c;
      else in_packet = false;
    }
  }
}

void readEstop() {
  unsigned long now = millis();
  bool raw = digitalRead(ESTOP_PIN) == LOW;
  if (raw != estop_raw_last) { estop_raw_last = raw; estop_change_ms = now; }
  if (raw != estop && now - estop_change_ms >= DEBOUNCE_MS) {
    estop = raw;
    reportEstop();
  }
  if (now - estop_report_ms >= ESTOP_REPORT_MS) reportEstop();
}

void render() {
  unsigned long now = millis();
  const uint32_t RED = strip.Color(255, 0, 0);

  // e-stop button overrides everything, even if Jetson is dead
  if (estop) { fill(RED); return; }

  if (state == 0 || now - last_cmd_ms > CMD_TIMEOUT_MS) {
    fill((now / 1000) % 2 ? strip.Color(255, 160, 0) : 0);   // yellow 0.5 Hz
    return;
  }

  switch (state) {
    case 'I': fill(strip.Color(0, 255, 0)); break;
    case 'P': {                                               // 3 s breathing
      unsigned long t = now % 3000;
      uint8_t v = t < 1500 ? t * 255 / 1500 : (3000 - t) * 255 / 1500;
      v = (uint16_t)v * v / 255;                              // gamma-ish
      fill(strip.Color(0, 0, v < 8 ? 8 : v));
      break;
    }
    case 'E': fill((now / 250) % 2 ? 0 : RED); break;         // 2 Hz
    case 'S': fill(RED); break;
  }
}

void setup() {
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  Serial.begin(BAUD);
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.clear();
  strip.show();
  estop = estop_raw_last = digitalRead(ESTOP_PIN) == LOW;
  Serial.print(F("LED_NANO READY\n"));
  reportEstop();
}

void loop() {
  readSerial();
  readEstop();
  render();
}
