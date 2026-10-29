# 한이음 프로젝트 — 메카넘 케어로봇 (Jetson Orin Nano, ROS2 Humble)

담당: 구영인 (SLAM · Nav2 · EKF · 시리얼브리지 · TF)
목표: SLAM 지도 생성 → Nav2 자율주행 → 장애물 회피 → 데모 영상

## 로봇 구성
- Jetson Orin Nano (ROS2 Humble) + Arduino UNO(클론, CH340) + ESP32-S3 #1(IMU+리프트) + ESP32 #2(섀시 가변폭, 업링크 미구현)
- 라이다: RPLIDAR A2M12, `/dev/ttyUSB0`
- UNO: `/dev/ttyCH341USB0` (재연결 시 번호 바뀔 수 있음 — 항상 `ls /dev/ttyCH341USB*`로 확인)
- ESP1(IMU): `/dev/ttyACM0` — **9/5 실장착 완료**. 칩은 ESP32-S3(QFN56, PSRAM 8MB), USB-시리얼은 WCH CH9102 계열(1a86:55d3)로 잡힘.
  ~~기본 launch 인자 `/dev/ttyUSB1`은 틀렸으니 항상 명시~~ → **10/4 기본값을 `/dev/ttyACM0`로 수정함.** 번호가 바뀌면 그때만 명시.
- 뎁스캠(RealSense D415)은 **9/5 기준 실제로 인식됨** (`Device with serial number ... was found`) — 예전 "미장착" 메모는 낡은 정보였음. 필요시 재확인.
- IMU(ESP1) **9/5 연결 완료 + 검증 완료**. 아래 §IMU 절 참조.

## ★ 10/4 IMU 경로 점검·수정 (윈도우 사본에서 작업, 젯슨 미반영 — 아래 §10/4 적용 절차)

9/29 실차 주행이 IMU 없이 돈 것으로 보여 IMU→EKF 경로 전체 점검. 발견·수정:
1. **IMU yaw가 EKF에서 통째로 버려지고 있었음** — 브리지가 `orientation_covariance[0] = -1`을 보냄.
   ROS 규약상 "orientation 없음"이라 robot_localization이 orientation을 무시 → 9/5 이후에도
   실제로는 gyro_z(vyaw)만 융합됐음. → roll/pitch 분산 1e3, yaw 0.01로 수정.
2. **ESP1 조용한 강등** — `esp1_port` 기본값(ttyUSB1)이 틀려서 인자 빼먹으면 ESP1을 못 열고
   경고 한 줄만 찍고 IMU 없이 계속 돌았음. 휠 vyaw는 9/5에 꺼뒀으므로 이러면 EKF 회전 정보 0.
   → 기본값 `/dev/ttyACM0`, `enable_esp1=true`인데 못 열면 **브리지 기동 실패**로 변경.
3. **IMU 워치독 신설** — `<I>` 패킷 1초 이상 끊기면 `IMU 수신 끊김` ERROR (기동 직후 8초 유예).
   5초 진단 로그에 `ESP1 IMU xxHz` 표시.
4. `ekf_real.yaml` `imu0_relative: true` (기동 시 정면 = yaw 0).
5. `lift.ino`: BNO055 **NDOF → IMUPLUS** (모터 근처 지자기 간섭 + 자북 기준 yaw 문제 제거),
   IMU 전송 **10Hz → 50Hz**, 기동 시 자이로 보정 대기(최대 3초, 이때 로봇 건드리지 말 것),
   `BNO055_ERROR` 1초마다 반복 출력.
6. `tools/test_bridge_watchdog.py` 갱신(String 스텁 누락으로 9/5 이후 깨져 있었음) — 33/33 통과.
7. `slam_real.launch.py` 주석의 `enable_esp1:=false` 절차 갱신.

수정분만 담은 패치: `imu_fix_1004.patch` (워크스페이스 루트).

### 10/4 적용 절차 (젯슨)
1. 젯슨 `~/hanium_ws` 코드가 윈도우 사본과 같은지 먼저 확인 (9/5 작업을 젯슨에서 했으므로)
2. src 동기화 + 빌드, `lift_esp/lift.ino`는 src 밖이라 따로 복사 →
   `/tmp/lift/lift.ino`로 두고 `arduino-cli compile/upload --fqbn esp32:esp32:esp32s3 -p /dev/ttyACM0 /tmp/lift`
3. raw 시리얼로 `BNO055 OK (IMUPLUS)`, `GYRO CALIB = 3`, `<I,...>` 50Hz 확인
4. bringup → `ESP1 IMU ~50Hz` 로그, `IMU 수신 끊김` 없음 확인
5. 손으로 반시계 회전 시 `/imu/data angular_velocity.z > 0`, 90° 회전 시 yaw 변화 ≈ 1.57
6. 90도 회전 테스트(`/odometry/filtered`) → SLAM → Nav2
7. IMU 장착 방향이 기판 x=전방, z=위가 아니면 URDF `imu_joint` rpy 수정

### 10/4 진행 상황 — bringup 재확인 + 바퀴 구동 문제 해결 (9/5 미해결 항목)

- 허브 빼고 라이다/ESP1/UNO 젯슨 직결, 포트: ESP1 `/dev/ttyACM0`, 라이다 `/dev/ttyUSB0`, UNO `/dev/ttyCH341USB0`.
  UNO에 또 PS2 디버그 스케치가 올라가 있어서 `smart_upcare_uno.ino` 재업로드함(§UNO 오작동 사고 패턴 재발 — 업로드할 때마다 확인 필요).
- 젯슨 전원을 충전기 → 보조배터리로 전환 후 재부팅, bringup 재확인: UNO 25.0Hz 100% good, ESP1 IMU 50.0Hz 100% good,
  `track_width` 파라미터 0.1354 반영 확인(`ros2 param get` 재확인 습관 유지).
- **9/5 "⚠️ 미해결 — 바퀴 구동 문제(배터리 의심)" 해결됨 — 배터리/하드웨어 문제가 아니었음.**
  `ros2 topic pub -1 /cmd_vel_smoothed ...`(1회성)으로 테스트했을 때는 바퀴가 전혀 안 돌았는데,
  원인은 **300ms 명령 워치독(`cmd_timeout`) + PWM 데드밴드 바닥(`deadband_pwm=25`/255)** 때문에
  신호가 너무 약하고(duty ~10%) 너무 짧게(최대 0.3초)만 나간 것이었음.
  `-r 10`으로 2~3초 지속 발행했더니 바퀴 4개 정상 회전(좌우 반대 방향, 소리도 정상) 확인됨.
  → **앞으로 cmd_vel 수동 테스트는 `-1` 단발 대신 `-r <Hz>`로 최소 1~2초 지속 발행할 것.**
  단발 테스트로 "안 움직인다"고 배터리/모터 탓하기 전에 이 캐비앗부터 의심할 것.
- 회전 명령(wz=0.3) 시 역기구학 계산 결과도 확인: 좌측 2바퀴/우측 2바퀴가 반대 부호로 나와 회전 패턴 정상,
  실측으로도 좌우 반대 방향 회전 확인됨 — `wheel_sign=[-1,-1,1,1]`이 회전 명령에서도 정상 동작함(배선 역전 보정이
  모션 타입과 무관하게 일관되게 적용되는 구조라 이론적으로도 맞음).
- **90도 회전 테스트 — PASS.** 바닥에 내려놓고 teleop(`j`)으로 반시계 회전, `/odometry/filtered` yaw
  93.69°→-175.50°(wrap 보정 시 +90.81°), 목표 90°와 오차 ~0.8° — IMU/EKF 회전 추적 정상 확인.
  - **1차 시도(`ros2 topic pub -1`, 단발) 실패, 2차(`-r 10`, 2.5초 지속) 실패, 3차(teleop, 충분히 길게) 성공.**
    → 제자리 회전은 PI 적분이 정지마찰 넘길 만큼 쌓이는 데 수 초가 걸리는 구조라, **짧은 명령으로는
    절대 안 돌고 충분히 길게(최소 몇 초) 지속해야 돈다.** 다음부터 바퀴/회전 테스트할 때 이 점 감안할 것
    (안 움직인다고 바로 배터리/하드웨어 의심하지 말고 먼저 지속시간부터 늘려볼 것).
  - teleop 쓸 때 **긴 명령을 복붙하면 중간에 개행이 끼어 일부만 먼저 실행되고 나머지가 teleop 키 입력으로
    먹혀버리는 사고**가 있었음(remap 플래그 `-r cmd_vel:=/cmd_vel_smoothed`가 빠진 채로 3번 연속 재발) →
    복붙 대신 손으로 한 줄로 쭉 타이핑하고 끝에 Enter 한 번만 누를 것.
  - **IMU 검증 전체 완료 — 매핑 진행 가능.**

### 10/4 bwd_gain 재튜닝 (직진 비대칭 보정 재발 → 해결)

매핑 주행 중 "직진하면 한쪽으로 휜다"는 8/16에 `bwd_gain=1.25`로 1차 보정했던 문제가 재발 확인됨.
`wheel_sign`([-1,-1,1,1])은 정상 반영돼 있었음(launch 덮어쓰기 버그 아님) — 순수 게인 재튜닝 필요.

- **`serial_bridge_node`에 `bwd_gain` 핫리로드 콜백 추가** (`add_on_set_parameters_callback`) —
  재시작 없이 `ros2 param set /serial_bridge_node bwd_gain <값>`으로 바로 튜닝 가능해짐.
  `robot_bringup.launch.py`에도 `bwd_gain` 인자 추가(기본값은 항상 노드 기본값과 동일하게 유지할 것 —
  안 그러면 track_width 때와 같은 "launch가 소스 기본값을 덮어쓰는" 사고 재발).
- **튜닝 방법**: 0.1m/s로 1m 전진 → `/imu/data` yaw 변화량 기록 → 1m 후진(원위치) → yaw 변화량 기록.
  기준: yaw가 늘어나면(반시계) `bwd_gain` 내리고, 줄어들면(시계) 올리기.
- **결과**: 1.25(전진 +19.4°, 후진 +77.6°/1m) → 1.125(+8.9°/+10.4°) → **1.0625(+6.0°/+6.2°, 확정)**
  → 1.03125까지 더 내려보니 전진이 반대로 오버슈트(-17.5°)하고 후진은 오히려 더 벌어져서(+11.4°)
  1.0625에서 멈춤. 이 미세 조정 구간은 바닥 마찰/시작-정지 타이밍 노이즈가 커서 더 쪼개도 의미 있는
  개선을 기대하기 어려움.
  - **중요 관찰**: `bwd_gain=1.25`일 때 전진(+19.4°)과 후진(+77.6°)이 크기가 많이 다르고, 게인을 더
    내려가니 전진/후진이 서로 반대 방향으로 벌어지기 시작함 — `bwd_gain`은 코드상 "양수 PWM" 쪽만
    부스트하는데, `wheel_sign` 부호 때문에 전진/후진마다 부스트받는 **물리적 바퀴가 서로 다름**
    (전진=우측, 후진=좌측 식). 즉 단일 `bwd_gain`으로 전진/후진을 동시에 완벽히 대칭화하는 건
    구조적으로 한계가 있을 수 있음 — 나중에 더 정밀한 보정이 필요하면 좌우 바퀴별 FF 게인을
    분리하는 쪽을 검토할 것.
- 재튜닝 중 **좀비 bringup 프로세스 사고**도 겪음: 팀원이 다른 노트북에서 같은 Jetson에 SSH로
  bringup을 띄워놓고 안 끈 채로 있었음(`enable_esp1:=false`로, pts/5, tmux 밖). 이게 시리얼
  포트를 점유해서 tmux 0번 bringup의 `/odom_raw`/`/imu/data`가 안 나오는 것처럼 보였음.
  좀비만 SIGINT로 끄려다가 라이다 포트 공유 때문에 tmux 0번 bringup까지 같이 죽는 부작용 있었음
  (rplidar가 SIGINT/SIGTERM에 응답 안 해서 SIGKILL까지 올라가며 "모터 정지" 정상종료 루틴을
  못 타서 라이다가 꺼진 뒤에도 계속 돌았음 → `python3 -c "import serial; ...'\xA5\x25'..."`로
  RPLIDAR STOP 명령 직접 전송해서 수동 정지). **교훈: 같은 Jetson을 여러 사람이 SSH로 공유할 때
  `ps aux`로 남의 bringup이 떠 있는지 먼저 확인할 것, 좀비 정리 시에도 tmux 0번과 라이다/시리얼
  포트를 공유하므로 영향 갈 수 있음을 감안할 것.**

- **teleop 회전 속도 관련 — 코드 버그 아니었음**: `turn=0.3rad/s`로 매핑 주행하다 "회전이 아예 안
  된다"(소리만 남)는 문제 재발. `moveBindings`(`j`=+1, `l`=-1) 자체는 정상, 원인은 FF
  (`ff_gain*target`)가 데드밴드(25)에도 못 미쳐 정지마찰을 못 넘긴 것 — `wz=0.3`→target≈1.25rad/s
  →FF≈18.7(<25), `wz=1.0`→target≈4.16→FF≈62(여유 있게 넘음). 안전 속도를 **직진 0.3m/s / 회전
  0.7rad/s**(FF≈44, 데드밴드 여유)로 상향 조정, `~/teleop_slow.sh`(리포 밖 파일) 갱신.
  **다음부터 회전이 "안 먹는다"고 하면 먼저 turn 속도가 데드밴드를 넘는지부터 의심할 것.**
- 다음: rosbag 녹화 + SLAM 매핑 → Nav2.

### ⚠️ 10/4 세션 마무리 — BNO055 I2C 재발성 락업 발견, 매핑 미완료로 종료

IMU 검증(90도 테스트 PASS) 이후 매핑 시도 중 **BNO055가 실주행 중 두 번이나 죽는 문제**를
발견해서 오늘은 매핑을 못 끝내고 여기서 정리함. **다음 세션 최우선 과제는 배선 점검/개선.**

**증상 2종류 (둘 다 bringup 재시작 없이 실주행 도중 발생)**:
1. (run12) 패킷은 50Hz로 계속 들어오는데 `yaw`/`gyro_z` 값이 전부 정확히 0.0 고정 —
   워치독이 못 잡음(패킷 수신 자체는 끊기지 않아서). `/imu/data` 직접 echo로만 발견됨.
2. (run14) 패킷 수신 자체가 완전히 멈춤(`ESP1 IMU 0.0Hz`) — 이번엔 `IMU 수신 끊김` 워치독이
   정상적으로 잡아냄. 더 심한 고장 형태.

**공통점**: 둘 다 로봇이 **바닥에서 수 분간 실주행 중**(모터 부하 있음, 방향 전환 반복)에
발생. 매핑 전 90도 회전 테스트(손/teleop) 때는 멀쩡했음.

**임시 복구법**: ESP1(ttyACM) **USB 케이블 뽑았다 꽂기**(=전원 재인가) — 소프트웨어 재시작만으론
안 됨, BNO055 I2C 락업은 전원을 끊어야 풀림. **주의: 재연결하면 포트 번호가 바뀔 수 있음**
(이번에 `ttyACM0`→`ttyACM1`로 바뀜) — 재연결 후 꼭 `ls /dev/ttyACM*` 재확인하고
`esp1_port:=` 인자로 명시해서 bringup 켤 것.

**원인 진단 시도 — 결론 못 냄**: 모터 EMI(전자파 간섭)가 I2C를 깨운다는 가설로, 바퀴 띄운
상태에서 강한 PWM(전진 0.3, 회전 wz=1.5)을 8초씩 걸며 IMU 노이즈 패턴을 관찰했으나
**재현 안 됨** — 멀쩡했음. 근데 이건 무부하·짧은(8초) 테스트라 실제 고장(바닥 주행·수 분·
반복 방향전환)보다 훨씬 약한 조건이라 가설을 기각한 건 아님. 그냥 결론을 못 낸 것.

**다음 세션 최우선 — 배선 점검/개선** (사람이 직접 봐야 함):
1. SDA/SCL 배선이 모터·모터드라이버 전선과 나란히/가깝게 지나가는지 육안 확인
2. **SDA/SCL 두 선을 서로 꼬아서(twisted pair) 배선** — 가장 간단하고 효과적인 1차 조치
3. I2C 배선을 모터 전선과 최대한 떨어뜨리고, 교차해야 하면 평행 말고 직각으로
4. ESP32 GND ↔ 모터드라이버 GND 공통 접지 상태 확인
5. BNO055 보드 VCC-GND 사이 0.1µF 디커플링 콘덴서 유무 확인, 없으면 추가
6. (여유 되면) I2C 선을 쉴드케이블 또는 알루미늄 호일로 감싸고 **한쪽 끝만 GND 접지**
   (양쪽 접지하면 그라운드 루프 생겨 역효과)
7. 배선 개선 후 **바닥에서 수 분간 실주행하며 IMU 모니터**(아래 `yaw_watch.py` 만들 것)로
   재발 여부 확인 — 8초 무부하 테스트로는 안 잡히므로 반드시 실조건으로 검증
8. (여유 되면, 소프트웨어 보완) `lift.ino`에 I2C 에러 감지 + `bno.begin()` 자동 재초기화
   로직 추가 검토 — 지금은 getEvent/getVector 결과를 에러 체크 없이 그대로 내보냄

**오늘 녹화한 rosbag 현황** (`~/hanium_ws/run*`, 전부 보존함):
| run | 길이 | IMU 상태 | 비고 |
|---|---|---|---|
| run7 | - | - | 팀원이 다른 노트북에서 띄운 것(enable_esp1:=false), 무관 |
| run8 | - | - | teleop remap 디버깅 중 생긴 것, 매핑용 아님 |
| run9 | - | - | 회전 안 됨(turn=0.3 데드밴드 문제) 확인용, 매핑 안 함 |
| run10 | - | - | 중간에 재시작 요청으로 폐기 |
| run11 | 80s | **정상** | SLAM 성공(`maps/my_map_v7`), 근데 너무 짧아서 좁음 |
| run12 | 180s(트림) | **고장(값 0 고정)** | SLAM 결과 `maps/my_map_v8` — 부채꼴로 깨짐, **쓰지 말 것** |
| run13 | - | - | 주행 중 "망함"으로 중단, 재시도함 |
| run14 | 137s | **고장(수신 끊김)** | SLAM 안 돌림(IMU 메시지 0개라 의미 없음) |

**지도 현황**: `maps/my_map_v7`(run11 기반)이 지금까지 중 유일하게 멀쩡한데 5.8×4.35m로
좁음. `maps/my_map_v8`(run12 기반)은 깨진 지도라 Nav2에 쓰면 안 됨. **다음 세션 목표**:
배선 고친 뒤 좀 더 넓게 다시 매핑해서 `my_map_v9`로 교체.

### 다음 세션 체크리스트 (갱신 — 10/4 종료 시점)
1. **배선 점검/개선부터** (위 §배선 점검 항목 1~6) — 매핑 재시도 전에 반드시
2. bringup 켜고 `ls /dev/ttyACM*`로 포트 확인 후 `esp1_port:=` 명시해서 기동
   (재연결 때마다 번호 바뀔 수 있음, 이번 세션에 `ttyACM0`→`ttyACM1` 실제로 겪음)
3. `ros2 param get /serial_bridge_node bwd_gain` → `1.0625` 확인 (10/4 재튜닝 확정값)
4. 바닥에서 몇 분간 실주행하며 IMU 모니터로 락업 재발 여부 먼저 확인 — **재발하면 매핑
   진행하지 말고 배선을 더 고칠 것** (반쯤 고장난 상태로 매핑해봐야 또 버림)
5. IMU 안정적이면 → rosbag 녹화(run15~) → SLAM → `my_map_v9` 저장 → Nav2
6. teleop은 `~/teleop_slow.sh`(speed=0.3, turn=0.7) 그대로 사용, 복붙 말고 직접 타이핑할 것
   (복붙 중 개행 끼어서 remap 날아가는 사고 여러 번 있었음)

## 현재 상태 (2026-09-05 기준)
- UNO 통신: 정상 (25Hz) — 단, 아래 §UNO 오작동 사고 참조 (한때 완전히 죽었다가 재플래시로 복구)
- ESP1(IMU): 정상, 패킷 100% 성공 (`ok=1050 bad=0`)
- EKF: `/odometry/filtered` 정상 발행 (~25Hz), `/odom_raw` + `/imu/data` 둘 다 정상 구독/융합 중
- SLAM: 아직 재시도 전 (오늘 IMU 작업에 시간 다 씀). 다음 세션에서 진행.
- Nav2: 미시도.
- ~~⚠️ 미해결 — 바퀴 구동 문제 (배터리 의심)~~ → **10/4 해결됨, 배터리/하드웨어 문제 아니었음.**
  원인은 단발(`-1`) cmd_vel 테스트가 300ms 워치독 + PWM 데드밴드에 걸려 너무 약/짧았던 것.
  지속 발행(`-r`)으로 재테스트하니 좌우 반대 방향 정상 회전 확인. 상세는 §10/4 진행 상황 참조.

## ✅ IMU(ESP1) 활성화 — 9/5 완료

### 발견했던 문제와 수정 내역
1. **ESP1 펌웨어가 프로토콜과 다름** (`lift_esp/lift.ino`): 마커 없이 `<yaw_deg,gyro_dps>` 2필드만 전송,
   각도 단위도 도(°)였음(ROS/EKF는 라디안 기대). → `readAndSendIMU()`를 고쳐 `<I,yaw_rad,gyro_rad,lift_mm,state>`
   형식으로 변경 (DEG_TO_RAD 곱함). `state`는 `currentStateChar()`로 H/M/T/- 계산해서 실음.
   `lift_mm`은 스텝→mm 환산 상수 미확정이라 **0.0 placeholder**로 채워둠(TODO, §lift_mm 환산 참조).
   `<D,state,checksum>` 완료 이벤트는 **아직 미구현** — coarse_state 자가복구 경로만으로 `/lift/state`가
   충분히 갱신되므로 급하지 않음. 여유 될 때 `lift.ino`에 엣지 검출 추가할 것.
2. **`serial_protocol.py`/`serial_bridge_node.py` 프로토콜 v3로 갱신** (리프트 담당 팀원 패치, 검증 후 병합):
   `UPLINK_SPEC['I']=4` (state 필드 추가), `'D':2` 신설. `on_esp1()`이 `pkt['type']`로 'I'/'D' 분기하도록 수정
   — **이 분기가 없으면 `<D,...>` 패킷 수신 시 `on_esp1`이 `pkt['yaw']` KeyError로 크래시 → ESP1 리더
   스레드 전체가 조용히 죽는 버그가 있었음(사전에 코드리뷰로 발견, 병합 전에 이미 수정된 버전으로 반영함)**.
   `/lift/state`(String) 퍼블리셔, `/lift_cmd_mm` 구독→`P<mm>` 다운링크 전송도 이번에 같이 반영.
3. **`track_width` 미반영 버그 — 원인 확정 + 수정** (`launch/robot_bringup.launch.py` 원래 51행):
   `DeclareLaunchArgument('track_width', default_value='0.20699')`가 소스 코드 기본값을 항상 덮어쓰고
   있었음. 소스(`serial_bridge_node.py`, `mecanum_kinematics.py`)를 아무리 고쳐도 이 launch 인자가
   이겨서 반영이 안 됐던 것. **launch 파일 기본값을 `0.1354`로 수정**. 앞으로 `ros2 param get` 확인은
   여전히 습관화할 것 — 이번엔 launch였지만 다음엔 다른 경로일 수 있음.
4. **`ekf_real.yaml` — `odom0_config`의 vyaw(index 11)를 `true→false`로 원복**. 8/29엔 IMU 미연결이라
   휠오도메트리 각속도로 yaw를 임시 대체했었는데, 이제 IMU가 붙었으니 yaw/vyaw는 `imu0_config`
   (index 5, 11 둘 다 이미 `true`)에 전담시킴 — 메카넘 슬립에 더 강함(명세서 §3.3).

### 검증 결과
- ESP1 raw 시리얼 직접 프로브: `<I,0.0011,0.0011,0.0,H>` 형태로 정상 (마커 O, 라디안 범위, state=H 정확)
- `ros2 topic hz /odom_raw` : ~25Hz
- `ros2 topic echo /imu/data --once` : orientation/angular_velocity 정상 값
- `ros2 topic hz /odometry/filtered` : ~25Hz (EKF 정상 융합 확인)
- **아직 못 한 것**: 90도 회전 테스트로 z≈0.707/w≈0.707 실측 검증 — 바퀴 구동 이슈로 중단.
  **다음 세션 최우선 항목.**

### gyro_z 단위 관련 미확인 사항
Adafruit_BNO055 라이브러리의 `getVector(VECTOR_GYROSCOPE)`는 보통 deg/s(dps) 단위로 알려져 있어
`lift.ino`에서 DEG_TO_RAD를 곱해뒀지만, **실측으로 재확인은 안 한 상태**. 90도 회전 테스트 때
`imu0_config`의 vyaw(각속도)도 같이 fusion되므로, 회전 중 `/imu/data`의 `angular_velocity.z`
피크값이 대략 몇 rad/s로 나오는지 봐서 단위가 맞는지 감으로도 확인할 것 (사람이 손으로 90도를
1~2초에 돌리면 대략 1~1.5 rad/s 근처가 정상 범위).

## ⚠️ UNO 오작동 사고 기록 (9/5) — 원인 두 가지가 겹쳐 있었음

1. **UNO가 USB 버스에서 완전히 사라짐** (`lsusb`에서도 안 보임). ESP1(같은 4포트 허브 공유)
   펌웨어 업로드 도중 허브에 순간 전원 변동이 있었던 것으로 추정. → **케이블 재연결로 복구됨**
   (커널이 장치 자체를 놓쳐서, 포트 재확인/replug 없이는 소프트웨어로 복구 불가했음).
2. **UNO에 엉뚱한 펌웨어가 올라가 있었음**: 정본 `arduino/smart_upcare_uno/smart_upcare_uno.ino`
   (`<E,..>`/`<M,..>` 프로토콜) 대신, **`2026ESWContest_free_SMARTUPCARE/Mecanum4WD.ino`라는
   PS2 컨트롤러 디버그 스케치**가 올라가 있어서 엔코더 자동 스트리밍이 꺼진 상태였음
   ("Jetson Auto Mode"를 PS2 TRIANGLE 버튼으로 켜야 하는 구조 — 시리얼 자동 송신 자체가 안 됨).
   → `arduino-cli`로 정본 재업로드해서 해결, `/odom_raw` 25Hz 정상 확인.
   **교훈: UNO가 이상하면 raw 시리얼로 부팅 배너부터 찍어볼 것** — 엉뚱한 펌웨어인지 바로 티남.

### 젯슨에 Arduino 툴체인 설치 완료 (9/5)
`arduino-cli` (`~/bin/arduino-cli`)에 아래가 이미 설치되어 있음 — 다음부터는 재설치 없이 바로 사용 가능:
- `arduino:avr` (UNO용, 원래 있었음) — FQBN `arduino:avr:uno`
- `esp32:esp32` (ESP32-S3용, 9/5 신규 설치) — FQBN `esp32:esp32:esp32s3`
- 라이브러리: Adafruit BNO055 / Adafruit Unified Sensor / Adafruit BusIO
- 컴파일 시 스케치 폴더명이 `.ino` 파일명과 같아야 함(arduino-cli 제약) — `lift_esp/lift.ino`처럼
  이름이 안 맞으면 `/tmp/어딘가/lift/lift.ino`로 복사해서 그 폴더를 컴파일 대상으로 줄 것.
- 컴파일: `arduino-cli compile --fqbn <FQBN> <스케치폴더>`
- 업로드: `arduino-cli upload -p <포트> --fqbn <FQBN> <스케치폴더>`

## ⚠️ 지금 막혀있는 문제 — track_width 미반영 버그 → **9/5 해결됨** (위 §IMU 활성화 3번 참조)

## 파일 위치 참고 (grep 결과)
track_width가 등장하는 파일들:
- `src/smart_upcare_control/smart_upcare_control/serial_bridge_node.py` (파라미터 선언, 실제값은 launch가 주입)
- `src/smart_upcare_control/smart_upcare_control/mecanum_kinematics.py` (기본값, 9/5 `0.1354`로 정정)
- `src/smart_upcare_control/launch/robot_bringup.launch.py` (★ 실제 반영되는 곳. 9/5 `0.1354`로 정정)
- `src/smart_upcare_control/launch/drive_test.launch.py` — 아직 `0.20699` 기본값일 수 있음, 쓸 때 확인
- `src/smart_upcare/urdf/smart_upcare.urdf.xacro`, `src/smart_upcare/launch/spawn.launch.py` — 시뮬/URDF용, 실차 odom 정확도와 무관

## SLAM 매핑 표준 절차

### 터미널 1 — bringup (IMU 켠 버전)
```bash
ls /dev/ttyCH341USB*
ls /dev/ttyACM*
cd ~/hanium_ws && source install/setup.bash
ros2 launch smart_upcare_control robot_bringup.launch.py \
    uno_port:=/dev/ttyCH341USB0 use_virtual_lift:=true
# (10/4부터 esp1_port 기본값 /dev/ttyACM0, enable_esp1 기본 true. 포트 번호 바뀌었을 때만 명시)
```
성공 신호: `UNO 25.0Hz ok=xxxx bad=0 (100.0% good)` **그리고** `ESP1 IMU ~50Hz ok=xxxx bad=0 (100.0% good)`.
`IMU 수신 끊김` ERROR가 보이면 **주행/매핑 중단** — IMU 없이 돌면 회전이 전혀 안 잡힘.
`엔코더 업링크 끊김` 에러가 뜨면 UNO 재확인부터(§UNO 오작동 사고 기록 참조).

### 90도 회전 테스트 (매핑 전 필수, 아직 미완료 — 다음 세션 최우선)
```bash
ros2 topic echo /odometry/filtered --field pose.pose.orientation
```
로봇을 벽에 평행하게 정렬 → 시작값 기록 → 인접 벽면과 평행해질 때까지 회전 → 종료값 기록.
각도 계산: `degrees(2*atan2(z,w))`, 두 값의 차이가 실제 회전각과 일치해야 정상.
teleop으로 돌릴 거면: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_smoothed`
(**teleop 창에 마우스로 클릭해서 포커스 반드시 줄 것** — 포커스 없으면 키 입력이 씹혀서 아무것도 안 나감.
`ros2 topic echo /cmd_vel_smoothed`로 실제 발행되는지 먼저 확인하는 습관 들일 것)

### rosbag 녹화 + 키보드 주행
```bash
# 터미널 2
cd ~/hanium_ws && ros2 bag record -o run_N /scan /tf /tf_static /odometry/filtered /odom_raw /joint_states /imu/data
# 터미널 3 (rosbag 시작 후 실제 주행)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_smoothed
```
키: i전진 ,후진 j좌회전 l우회전 k정지, z로 감속. 천천히, 짧게 끊어서, 회전 최소화, 벽 따라 한 바퀴, 출발점 복귀.

### SLAM (rosbag 리플레이로 여러 번 시도 가능)
```bash
# 터미널 2 (재사용)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
# 터미널 4
ros2 bag play run_N --clock -r 0.5
```

### 맵 저장 (이 옵션 없으면 실패함)
```bash
mkdir -p ~/hanium_ws/maps
ros2 run nav2_map_server map_saver_cli \
    -f ~/hanium_ws/maps/my_map_vN \
    --ros-args -p map_subscribe_transient_local:=true
```

## Nav2 자율주행 (맵 완성 후)
```bash
# 터미널 2
ros2 launch smart_upcare_control navigation.launch.py \
    slam:=false map:=~/hanium_ws/maps/my_map_vN.yaml

# 초기위치 지정 (필수)
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
'{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}'

# 목표점 발행
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "map"}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}'
```

## 확정된 캘리브레이션 값
| 항목 | 값 | 비고 |
|---|---|---|
| wheel_radius | 0.0394 m | 직진 2m 실측 |
| bwd_gain | **1.0625** | 직진 비대칭 보정, 10/4 재튜닝(아래 §10/4 bwd_gain 재튜닝 참조) |
| wheel_sign | [-1,-1,1,1] | 좌측 2바퀴 배선 반대 |
| footprint (축소) | 0.189 × 0.140 반치수 | |
| inflation_radius | 0.24 | 외접원 235mm 기준 |
| track_width | **0.1354 m** | 9/5: launch 인자 버그 수정 완료, 실제로 반영됨. 90도 회전 테스트로 최종 검증 필요 |

## 자주 걸리는 함정
- `map_saver_cli`는 `--ros-args -p map_subscribe_transient_local:=true` 없으면 무조건 실패함 (`Failed to spin map subscription`)
- launch 인자로 파라미터 넘겨도 소스 기본값이 조용히 이길 수 있음 — 9/5에 `track_width`가 실제로 이 패턴이었음
  (원인: launch 파일 자체의 `DeclareLaunchArgument` 기본값). `ros2 param get`으로 실제 반영 여부 확인 습관화.
- 젯슨 재부팅/노드 재시작 후에는 반드시 `ros2 param get`으로 실제 반영 여부를 확인할 것 — "재시작했다"와 "새 값이 적용됐다"는 다른 문제임
- SSH가 WiFi 핫스팟(`yeongin`, autoconnect-priority 100으로 이미 등록됨) 통해 연결됨. IP는 매번 바뀔 수 있음 — `arp -a`로 확인
- **`pkill -f <패턴>`을 셸 명령어 안에서 쓸 때, 그 패턴 문자열이 지금 실행 중인 명령어 자체의 텍스트에도 포함되면
  pkill이 자기 자신(그리고 뒤에 이어지는 명령들)을 먼저 죽여버릴 수 있음.** `pkill -f '[s]erial_bridge'`처럼
  첫 글자를 대괄호로 감싸는 트릭으로 자기매칭을 피할 것 (9/5 브링업 재시작 중 이걸로 삽질함).
- UNO/ESP1/라이다가 같은 USB 허브를 공유 — 한쪽 펌웨어 업로드/재연결 시 다른 쪽에 순간 전원 변동이
  생겨 USB 버스에서 통째로 사라질 수 있음(9/5 실측). 이상하면 `lsusb`로 장치 자체가 보이는지부터 확인.
- **모터/배터리 이상 시 teleop 키보드가 안 먹는 것과 헷갈리지 말 것**: `ros2 topic echo /cmd_vel_smoothed`로
  명령이 실제로 나가는지 먼저 확인 → 나가는데도 안 움직이면 그때 배터리/배선/모터 쪽 문제로 좁혀갈 것.

## 다음 세션 체크리스트 (배터리 충전 후)
1. ~~배터리 충전 상태로 UNO/ESP1/라이다 전원 전체 재확인~~ — 10/4 완료(보조배터리로 전환, 정상)
2. ~~`ls /dev/ttyCH341USB*`, `ls /dev/ttyACM*`로 포트 재확인~~ — 10/4 완료 (ESP1 ttyACM0, 라이다 ttyUSB0, UNO ttyCH341USB0)
3. ~~bringup 재시작 → `track_width` 0.1354 반영 재확인~~ — 10/4 완료
4. ~~바퀴 직접 구동 테스트~~ — **10/4 완료, PASS.** 단발 테스트(`-1`)는 약해서 안 움직였던 것뿐이고,
   지속 발행(`-r 10`, 2~3초)으로 좌우 반대 방향 정상 회전 확인됨. 배터리/모터 문제 아니었음.
5. ~~90도 회전 테스트~~ — **10/4 완료, PASS** (오차 ~0.8°). §10/4 진행 상황 참조. **다음: SLAM → Nav2**
6. 브링업 재시작 전에 좀비 프로세스 정리 권장: `pkill -f '[r]obot_bringup.launch'; pkill -f '[s]erial_bridge';
   pkill -f '[e]kf_node'; pkill -f '[r]plidar'; pkill -f '[d]epth_to_scan'` (9/5에 rplidar/robot_state_publisher/
   depth_to_scan이 중복으로 남아있던 적 있음 — 젯슨 재부팅이 제일 깔끔함)
7. (10/4 신규 발견) RealSense D415가 bringup 중 `No such device`로 끊겼다 재시작을 반복 — SLAM은 라이다 기반이라
   블로커는 아니지만, Nav2 장애물회피에서 뎁스캠을 쓴다면 이 부분 재확인 필요.
