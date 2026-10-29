# 아두이노 구동부 통합 — 실차 검증 회신

> **From** 실차/펌웨어측 (아두이노 UNO 구동 · 시리얼)
> **To** 구영인 (젯슨 ROS2)
> **대상** `README_실차통합_핸드오프.md` §2 프로토콜 / §6 확정 필요 항목
> **기준일** 2026-07-27

핸드오프 문서대로 **UNO 펌웨어를 작성해 실차에서 통신·구동까지 검증**했습니다.
이 문서는 추가된 파일과 §6 블로킹 항목 회신입니다.
**라이다/카메라/Nav2/리프트 관련은 건드리지 않았습니다.**

---

## 1. 추가된 것 (구동부만)

| 경로 | 내용 |
|---|---|
| `arduino/smart_upcare_uno/` | ★ UNO 펌웨어 + 필요한 라이브러리 (Arduino IDE 로 폴더째 열어 업로드) |
| `src/.../smart_upcare_control/teleop_keyboard.py` | WASD 수동조종 (기존 워크스페이스에 수동조종 노드가 없었음) |
| `src/.../launch/drive_test.launch.py` | 구동부만 단독 검증 런치 (센서·Nav2 없이 UNO + 바퀴 + odom) |
| `serial_bridge_node.py` | **`enable_esp1` 파라미터 1개 추가** (아래 §3) — 그 외 로직 변경 없음 |
| `setup.py` | teleop 엔트리포인트 1줄 추가 |

---

## 2. §6-1 프로토콜 정합 — ✅ 일치 (펌웨어를 문서에 맞춤)

문서 §2 스펙 그대로 구현했습니다.

- 업링크 `<E,c1,c2,c3,c4>` — 누적 카운트, **25Hz 연속 스트리밍**
- 다운링크 `<M,pwm1,pwm2,pwm3,pwm4>` — -255~255, 부호=방향
- 휠 순서 c1=M1(FL) c2=M2(RL) c3=M3(RR) c4=M4(FR)
- baud 115200

검증: 펌웨어 출력 문자열을 `serial_protocol.FrameScanner` 에 그대로 넣어 파싱 성공,
청크 분할·쓰레기 혼입 복구까지 확인. `build_motor_downlink()` 출력도 펌웨어 파서 규칙과 일치.

## 3. §6 확인필요 ① Setspeed 16× 스케일 — ✅ 이중 적용 없음

펌웨어는 **별도 스케일을 곱하지 않습니다.**
QGPMaker 라이브러리의 `setSpeed(0~255)` 내부에서 PCA9685 12bit 로 ×16 변환이 일어납니다.
즉 젯슨이 보내는 **최종 PWM(-255~255)을 그대로 인가**하는 게 맞고, 이중 적용 없습니다.
→ 젯슨은 지금 가정대로 최종 PWM 만 내려보내면 됩니다.

## 4. §6 확인필요 ② 300ms 워치독 — ✅ 펌웨어에도 구현

`CMD_TIMEOUT_MS = 300`. 마지막 다운링크 수신 후 300ms 넘으면 4바퀴 자동 정지.
젯슨측 워치독과 **이중 방어** 됩니다.

## 5. §1 전제 "속도 제어는 젯슨" — ✅ 준수

펌웨어에 PID 없음(개루프). 받은 PWM 을 방향+크기로 인가만 합니다.
속도 폐루프(FF+PI)는 젯슨 `serial_bridge_node` 담당 그대로입니다.

---

## 6. ⚠️ 실차에서 바뀐 값 — 젯슨측 기본값 수정 필요

### 6-1. UNO 시리얼 포트 (중요)

실차 UNO 는 **CH340 칩 클론**이라 `/dev/ttyACM0` 로 안 잡힙니다.
젯슨 커널(5.15-tegra)에 드라이버가 없어서 **포트 자체가 안 생기는** 문제가 있었고,
아래 3단계를 모두 거쳐야 잡힙니다.

```bash
# ① WCH 드라이버 빌드 (커널에 ch341 미탑재)
git clone https://github.com/WCHSoftGroup/ch341ser_linux.git
cd ch341ser_linux/driver && make && sudo make install

# ② brltty 제거 (CH340 USB ID 를 가로채 포트를 강제로 끊음)
sudo apt remove -y brltty

# ③ 권한 (dialout) — 재로그인 필요
sudo usermod -aG dialout $USER
echo 'KERNEL=="ttyCH341USB[0-9]*", GROUP="dialout", MODE="0660"' | sudo tee /etc/udev/rules.d/99-ch341.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

결과 포트: **`/dev/ttyCH341USB0`**
→ `robot_bringup.launch.py` 의 `uno_port` 기본값 조정 또는 `/dev/serial/by-id/…` 고정 권장.
(정품 UNO 로 교체하면 `/dev/ttyACM0` 로 정상 인식됩니다)

### 6-2. 모터 방향 (wheel_sign)

전진 명령에 로봇이 제자리 회전하는 현상이 있었습니다 — **우측 2바퀴 배선 방향이 반대**입니다.
펌웨어는 raw 인가만 하므로 **젯슨 `wheel_sign` 에서 흡수**해 주세요.
실차 관측 기준 `wheel_sign: [1, 1, -1, -1]` 이 맞았습니다. (조립 최종본에서 재확인 필요)

---

## 7. 현재 상태 / 미연결

| 항목 | 상태 |
|---|---|
| UNO 엔코더 업링크 `<E,..>` | ✅ 25Hz 수신 확인 |
| UNO 모터 다운링크 `<M,..>` | ✅ 구동 확인 (WASD) |
| ESP32 #1 (BNO055 IMU + 리프트) | ❌ **미연결** — `<I,..>` 안 옴 |
| ESP32 #2 (섀시폭) | ❌ 미연결 |

**ESP32 미연결이면 `serial_bridge` 가 포트 열기 실패로 죽습니다** (`_open` 이 raise).
그래서 `enable_esp1` 파라미터를 추가했습니다. 기본값 `True` 라 **기존 동작은 그대로**이고,
ESP32 확보 전에는 `enable_esp1:=false` 로 UNO 단독 운용이 가능합니다.

> IMU 가 없으면 EKF 에 yaw 소스가 없어 heading 이 발산합니다.
> 그래서 `drive_test.launch.py` 에는 EKF 를 넣지 않았고, `/odom_raw` 로만 확인합니다.
> ESP32#1 붙는 즉시 `robot_bringup.launch.py` 정규 경로로 넘어가면 됩니다.

---

## 8. 실행 방법

### 8-1. 펌웨어 업로드
```bash
sudo chmod a+rw /dev/ttyCH341USB0          # 업로드 직전 (영구설정은 §6-1 ③)
arduino-cli compile -b arduino:avr:uno arduino/smart_upcare_uno
arduino-cli upload  -b arduino:avr:uno -p /dev/ttyCH341USB0 arduino/smart_upcare_uno
```
확인: `python3 -m serial.tools.miniterm /dev/ttyCH341USB0 115200`
→ `<E,0,0,0,0>` 가 25Hz 로 흐르면 정상. 바퀴를 손으로 돌리면 값이 변합니다.
(★ miniterm 과 ROS 는 포트를 동시에 못 씁니다. 확인 후 반드시 종료)

### 8-2. 구동부만 검증 (센서·Nav2 없이)
```bash
colcon build --packages-select smart_upcare smart_upcare_control
source install/setup.bash
ros2 launch smart_upcare_control drive_test.launch.py
```
- WASD 창에서 조종 (W/S 전후진, A/D 횡이동, Q/E 회전, Space 정지)
- 확인: `ros2 topic hz /odom_raw`, `ros2 topic echo /odom_raw`
- ★ 첫 실행은 **받침대에 올려 바퀴를 띄운 상태**로 방향부터 확인

### 8-3. 전체 브링업 (ESP32 확보 후)
```bash
ros2 launch smart_upcare_control robot_bringup.launch.py uno_port:=/dev/ttyCH341USB0
```

---

## 9. 남은 확정 필요 (펌웨어측 관점)

1. **리프트 다운링크 패킷** (§6-2) — `<L,target_mm>` 로 정하면 ESP32#1 펌웨어에 반영 가능. 포맷 확정 요청.
2. **`track_width`/`wheelbase` 실측** — 조립 완료 후 줄자 실측값으로 launch 인자 교체.
3. **`ff_gain`/`deadband_pwm` 실측** — PWM 계단입력 → 정상상태 각속도 맵 측정 필요.
   현재 오픈루프 구동만 검증했고, 폐루프 게인 튜닝은 미실시.
4. **`wheel_sign` 최종 확정** — 조립 최종본에서 바퀴별 전진 시 카운트 증가 방향 재확인.
