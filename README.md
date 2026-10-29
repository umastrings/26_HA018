# Smart Up-Care

메카넘 휠 케어로봇 자율주행 — 한이음 ICT 공모전

요양시설 복도처럼 좁고 특징이 적은 실내에서, 가변폭 섀시와 승강 리프트를 갖춘
홀로노믹 로봇이 자율주행하는 것을 목표로 한다.

---

## 저장소 구조

```
.
├── src/                        ROS 2 패키지 (젯슨에서 실행)
│   ├── smart_upcare/           URDF · Gazebo 월드 · RViz
│   └── smart_upcare_control/   런타임 노드 · 설정 · launch
├── arduino/
│   └── smart_upcare_uno/       UNO 펌웨어 (엔코더 + 모터 PWM)
├── tools/                      캘리브레이션 · 검증 도구 (ROS 불필요)
└── docs/                       핸드오프 · 체크리스트 · 진단기록
```

## 하드웨어 구성

| 역할 | 장치 | 담당 |
|---|---|---|
| 상위 제어 | Jetson Orin Nano (ROS 2 Humble) | 자율주행/퍼셉션 |
| 구동 | Arduino UNO — 엔코더 4채널 + 모터 PWM | 구동팀 |
| IMU + 리프트 | ESP32 #1 — BNO055 + 시저 리프트 | 리프트/구동팀 |
| 섀시폭 | ESP32 #2 — 스텝모터 (비블로킹) | 섀시팀 |
| 센서 | RPLIDAR A2M12 · Intel RealSense D415 | — |

**속도 제어는 젯슨이 담당한다.** UNO 는 개루프로 받은 PWM 을 인가만 하고,
엔코더 각속도를 피드백으로 한 FF+PI 폐루프는 `serial_bridge_node` 안에 있다.
(AVR UNO 는 micro-ROS 미지원 → PySerial 브리지로 결정. 근거는 `docs/` 참조)

## 시리얼 프로토콜

`<` `>` 프레이밍 + 첫 필드 타입 마커 + CSV. 정의 원본은
[`src/smart_upcare_control/smart_upcare_control/serial_protocol.py`](src/smart_upcare_control/smart_upcare_control/serial_protocol.py).

| 방향 | 출처/대상 | 패킷 |
|---|---|---|
| 업링크 | UNO | `<E,c1,c2,c3,c4>` 누적 엔코더 카운트 · 25Hz |
| 업링크 | ESP32 #1 | `<I,yaw,gyro_z,lift_mm>` |
| 업링크 | ESP32 #2 | `<W,width_mm>` (옵션) |
| 다운링크 | UNO | `<M,pwm1..4>` PWM −255~255 · 50Hz |

휠 순서는 전부 공통: `M1(앞좌/FL) · M2(뒤좌/RL) · M3(뒤우/RR) · M4(앞우/FR)`

> ⚠️ **엔코더는 정지 중에도 계속 보내야 한다.** `<E,..>` 가 200ms 넘게 끊기면
> 젯슨이 폐루프를 멈춘다 (`enc_timeout`). 피드백이 죽은 채로 PI 를 돌리면
> 적분항이 포화해 모터가 폭주하기 때문. 펌웨어 루프에 200ms 이상 블로킹이
> 없어야 한다.

## 실행

```bash
# 최초 1회
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select smart_upcare smart_upcare_control
source install/setup.bash

# ① 구동부만 (센서·Nav2 없이 UNO + 바퀴)
ros2 launch smart_upcare_control drive_test.launch.py

# ② 로봇 브링업 (센서 + 브리지 + EKF + TF)
ros2 launch smart_upcare_control robot_bringup.launch.py
#   ESP32 미연결이면:  enable_esp1:=false

# ③ 네비게이션
ros2 launch smart_upcare_control navigation.launch.py slam:=false map:=~/care_map.yaml

# ④ 데모 시퀀스
ros2 launch smart_upcare_control demo_scenario.launch.py
```

시뮬레이션(Gazebo)은 `robot_bringup` 대신 `sim_bringup.launch.py` 를 쓴다.
`sensors` / `serial_bridge` / `robot_bringup` 은 실차 전용.

### UNO 포트

실차 UNO 는 CH340 클론이라 `/dev/ttyACM0` 로 안 잡힌다. 젯슨 커널에 드라이버가
없어서 아래를 거쳐야 `/dev/ttyCH341USB0` 가 생긴다.

```bash
git clone https://github.com/WCHSoftGroup/ch341ser_linux.git
cd ch341ser_linux/driver && make && sudo make install
sudo apt remove -y brltty          # CH340 USB ID 를 가로챔
sudo usermod -aG dialout $USER     # 재로그인 필요
```

## 캘리브레이션

ROS 없이 UNO 와 직접 통신한다. `colcon build` 전에도 돌아간다.

```bash
pip3 install pyserial
python3 tools/calibrate_drive.py enc        # 감속비 + enc_sign
python3 tools/calibrate_drive.py motor      # wheel_sign
python3 tools/calibrate_drive.py step       # ff_gain + deadband
python3 tools/calibrate_drive.py watchdog   # 펌웨어 워치독
python3 tools/calibrate_drive.py report     # 파라미터 출력
```

절차는 [`docs/hanium_구동계_캘리브레이션_절차_0807.md`](docs/hanium_구동계_캘리브레이션_절차_0807.md).

## 테스트

둘 다 ROS·실물 없이 돌아간다. 코드 고치고 나서 먼저 이걸 돌릴 것.

```bash
python3 tools/test_bridge_watchdog.py    # 워치독 · enc_sign · 포트 (24개)
python3 tools/test_calibrate_drive.py    # 가짜 UNO 상대 캘리브레이션 (11개)
```

---

## 작업 규칙

**직접 `main` 에 푸시하지 않는다.** 브랜치를 파고 PR 로 합친다.

```bash
git switch -c drive/encoder-fix      # 또는 ros/..., firmware/..., docs/...
# 작업
git add -A && git commit -m "엔코더 read() 인터럽트 보호"
git push -u origin drive/encoder-fix
```

이유: 8/7 에 워크스페이스가 두 갈래로 갈라져 수동 병합이 필요했다.
`enable_esp1` 을 양쪽이 따로 구현했고, 한쪽엔 7/27 이전 치수가 남아 있었다.
브랜치+PR 이면 이런 게 합칠 때 눈에 보인다.

**공유 인터페이스를 바꾸면 반드시 알린다.**

- `serial_protocol.py` — 펌웨어와 젯슨 양쪽이 같이 바뀌어야 함
- `config/*.yaml` 파라미터 이름
- 토픽·프레임 이름

**커밋 메시지는 한국어로 무엇을/왜.** 나중에 심사에서 개발 과정을 설명할 때 그대로 쓴다.

**측정값은 커밋하지 않는다.** `calib_result.json` 은 `.gitignore` 에 있다.
확정되면 launch 기본값이나 yaml 에 반영하고, 그 커밋에 근거를 남긴다.

## 담당

| 영역 | 담당 |
|---|---|
| SLAM · Nav2 · EKF · 시리얼 브리지 · TF | 구영인 |
| UNO 펌웨어 · 모터 · 엔코더 · 배선 | 구동팀 |
| 리프트 기구 · ESP32 #1 펌웨어 | 리프트팀 |
| 가변폭 섀시 · 3D 프린트 | 섀시팀 |
| OCR · 음성 · LLM · UI · 낙상감지 | 부가기능 담당 |
