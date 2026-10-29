# 8/4 시뮬 진단 기록 — STEP 2 (병실 진입) 미완료

> 요약: **오늘 목표(침상 옆 도착)는 달성 못 했다.** 대신 문제의 위치를
> `controller_server` 출력단으로 좁혔고, 별개의 심각한 설정 결함 2건을 발견했다.
> 성능 문제(RTF 0.59 → 0.93)는 원인 규명 후 해결됐다.

---

## 1. 오늘의 경과

| # | 한 일 | 결과 |
|---|---|---|
| 1 | 8/3 개선 3종 적용 상태 확인 | yaml 반영 확인 |
| 2 | 침대 목표 좌표 재계산·표 갱신 | 월드 파일에서 직접 검산 완료 |
| 3 | 1차 주행 (RViz + 화면녹화) | 실패. RTF 0.59, 스캔 드롭 |
| 4 | CPU 실측 → AMCL 회귀 되돌림 | 원인 확정 |
| 5 | 2차 주행 (headless, rosbag) | RTF 0.93 ✓ / 주행은 world x≈7.1에서 실패 |
| 6 | 실패 로그 분석 | 감속 가설 기각, 회전 불가 조건 도출 |
| 7 | footprint 실측 불일치 발견 → 수정 | 회전 여유 ±3.8cm → ±22cm |
| 8 | 3차 주행 | **같은 자리(x≈7.10)에서 동일 실패** |
| 9 | 정지 중 cmd_vel 계측 | `/cmd_vel`·`/cmd_vel_smoothed` **둘 다 없음** |

---

## 2. 확정된 사실 (직접 측정 근거 있음)

### 2-1. 성능 병목은 시뮬레이터가 아니라 화면이었다

```
top (1차 주행 중, 4코어 VM)
  component_container  88.2%   Nav2
  rviz2                70.6%   ← 화면 그리기
  gnome-shell          47.1%   ← 데스크톱 합성 + 화면녹화 인코딩
  gzserver             29.4%   시뮬레이터
  Xorg                 17.6%
  %Cpu: us 38.2 / sy 32.7 / si 18.2 / id 10.9
  load average 24.43
```

**화면 관련 합계 136% > Nav2 88% > 시뮬 29%.**
VirtualBox에는 GPU 가속이 없어 RViz가 CPU로 소프트웨어 렌더링을 한다.
`si 18.2%`(소프트 인터럽트)도 RViz가 스캔·TF·지도를 루프백으로 수신하며 발생.

RViz·녹화 제거 후 **RTF 0.59 → 0.93** (gz stats 실측).

> **운영 원칙**: 주행 검증과 영상 녹화를 같은 실행에서 하지 말 것.
> 검증은 headless, 녹화는 별도 재생 세션.

### 2-2. 정지 지점은 3회 재현됐다 — 결정적 실패다

| 회차 | 설정 | 정지 world x | y |
|---|---|---|---|
| 1차 | AMCL 정밀도 상향 | 7.03 ~ 7.26 | −0.04 |
| 2차 | AMCL 되돌림 | 7.09 | −0.08 |
| 3차 | footprint 수정 | 7.10 | −0.23 |

설정을 세 번 바꿨는데 정지 지점이 0.07m밖에 안 움직였다.
**확률적 실패가 아니라 결정적 실패**이며, 튜닝으로 풀 문제가 아니다.

### 2-3. 정지 중 컨트롤러가 명령을 내지 않는다

```
timeout 5 ros2 topic hz /cmd_vel            → 출력 없음
timeout 5 ros2 topic hz /cmd_vel_smoothed   → 출력 없음
timeout 3 ros2 topic echo /cmd_vel --once   → 출력 없음
```

Collision Monitor가 차단한 것이라면 `/cmd_vel`은 나오고 `/cmd_vel_smoothed`만
없어야 한다. **둘 다 없다 = 체인의 최상류에서 이미 끊겨 있다.**

⚠️ **이 결론은 음성 측정에 의존한다. 내일 반드시 재검증할 것** (§6 참조).

---

## 3. 기각된 가설 (근거 포함)

| 가설 | 기각 근거 |
|---|---|
| Collision Monitor 감속이 원인 | 마지막 감속 메시지 t=713, 실패 t=886. **173초간 무관** |
| 위치추정(AMCL) 정밀도 부족 | 정밀도 상향 시 오히려 악화(CPU). 되돌려도 동일 실패 |
| 폴리곤이 벽을 물어 정지 | `/cmd_vel`이 아예 없음 → 폴리곤 통과 이전 단계 문제 |
| 침대 목표가 inflation 안에 있음 | 사실이었으나 별건. 복도 구간 실패와 무관 |
| MPPI 계산량 과다 | 1/4로 줄여도 동일 실패 (CPU는 개선됨) |
| 복도 폭 1.2m가 물리적 병목 | footprint를 6배 여유로 바꿔도 동일 실패 → 주원인 아님 |
| 월드에 장애물이 있음 | x 6.40~7.40 구간은 깨끗한 1.2m, 7.40부터 1.4m로 확장. 좌표 검산 완료 |

---

## 4. 발견한 설정 결함 (실패 원인과 별개로 반드시 고쳐야 함)

### 4-1. costmap footprint가 실제 로봇의 1.7배 ✅ 수정 완료

```
URDF 실측 (smart_upcare.urdf.xacro)
  섀시 0.20 × 0.20
  트랙폭 0.20699 + 바퀴폭 0.048        → 폭   0.255
  리프트 플랫폼 0.299 (지배적)          → 길이 0.299
  ⇒ 실제 0.299 × 0.255

기존 footprint  0.499 × 0.451   ← 실물 섀시(394×492mm) 펼침 기준값
수정 footprint  0.34  × 0.30
```

**영향** — Nav2는 벽에서 *내접원* 거리 안쪽을 무조건 충돌(cost 253)로 칠한다.
복도 반폭 0.60 기준 제자리 회전 가능 조건:

```
외접원 + |중앙이탈| < 복도반폭 − 내접원

기존:  0.336 + |y| < 0.3745  →  |y| < 0.038   (±3.8 cm)
수정:  0.227 + |y| < 0.45    →  |y| < 0.223   (±22 cm)
```

8/4 로그의 `[behavior_server]: Collision Ahead - Exiting Spin`은 실제 충돌이
아니라 과대 footprint로 코스트맵상 회전이 막힌 것이었다.

⚠️ **실물 섀시 완성 시 되돌릴 것.** 펼침 394×492mm에서는 복도 1.2m가 실제로
회전 불가이며, 그것이 가변폭 기구의 존재 이유다. 축소(280×378mm) 상태 회전
시나리오를 Nav2 footprint 동적 변경으로 구현해야 한다.

### 4-2. `velocity_smoother`와 `collision_monitor`가 충돌 ❗ 미수정 — 내일 1순위

`navigation.launch.py`가 `nav2_bringup/bringup_launch.py`를 그대로 include하는데,
여기에는 **`velocity_smoother`가 포함**되어 있다 (8/4 로그에서 활성화 확인).

```
velocity_smoother :  /cmd_vel  →  /cmd_vel_smoothed
collision_monitor :  /cmd_vel  →  /cmd_vel_smoothed     ← 동일 입출력
```

**두 노드가 같은 토픽에 동시 발행**하고, `planar_move`가 그걸 구독한다.

그리고 더 심각한 것: **`nav2_care.yaml`에 `velocity_smoother` 항목이 없다.**
즉 기본 파라미터로 동작한다. Nav2 `velocity_smoother`의 기본
`max_velocity`는 차동구동 기준 `[0.50, 0.0, 2.5]`로 알려져 있고,
**y 성분이 0.0이면 횡이동(strafe)이 전량 0으로 클램프된다.**

메카넘 홀로노믹의 핵심 기능이 조용히 죽는다는 뜻이다. MPPI에 `vy_max: 0.3`을
줘도 하류에서 잘린다. 좁은 복도에서 횡방향 보정을 못 하면 로봇은 회전으로만
정렬해야 하고, 이는 오늘 관측된 과도한 제자리 회전·진동과 정확히 일치한다.

> **미확인**: 설치된 Nav2 버전의 실제 기본값은 확인하지 않았다.
> 내일 `ros2 param get /velocity_smoother max_velocity`로 반드시 검증할 것.

---

## 5. 프로세스 평가 — 오늘 무엇이 잘못됐나

### 5-1. 신호 체인을 먼저 재지 않았다 (가장 큰 손실)

`/cmd_vel` vs `/cmd_vel_smoothed` 계측은 **13초**면 끝나고, 문제 공간을 서로
겹치지 않는 4갈래로 완전히 분할한다.

| /cmd_vel | /cmd_vel_smoothed | 원인 위치 |
|---|---|---|
| 있음 | 없음 | Collision Monitor |
| 없음 | 없음 | controller_server 상류 |
| 있음 | 있음(≈0) | 경로/목표 |
| 있음 | 있음(큼) | 물리/플러그인 |

이걸 **11번째로** 했다. 그 전에 Collision Monitor·AMCL·footprint 세 군데를
추측으로 건드렸고, 전부 헛다리였다.

> **원칙**: 증상을 보고 부품을 고르지 말고, **신호가 어디서 끊기는지 먼저 잰다.**
> 분기 테스트가 부품 수리보다 항상 먼저다.

### 5-2. 화면에 보이는 증상으로 진단했다

"빨간 스캔이 지도와 어긋남" → 위치추정 오차로 판단 → AMCL 정밀도 상향.

실제로는 RViz 로그에 답이 있었다:

```
Message Filter dropping message: frame 'lidar_link' ...
  the timestamp on the message is earlier than all the data in the transform cache
```

메시지가 버려지면 RViz는 마지막 성공 프레임을 계속 그린다. 즉 그 그림은
**오래된 화면**이지 위치추정 오차가 아니었다.

> **원칙**: CPU가 포화된 상태에서 화면상의 증상은 진단 근거가 될 수 없다.
> 부하부터 확인한다.

### 5-3. 한 번에 여러 개를 바꿨다

8/3에 3개를 동시 적용했고, 그중 AMCL 2건(`max_beams` 60→120,
`update_min_d` 0.25→0.15)은 AMCL 부하를 약 3.3배로 올리는 **성능 회귀**였다.
MPPI에서 아낀 만큼을 도로 먹었다. 되돌리고 CPU를 안 먹는 `alpha 0.05`만 남겼다.

> **원칙**: 변경은 하나씩. 효과가 없으면 되돌린 뒤 다음 것으로.

### 5-4. 잘한 것

- 감속 가설을 **로그 타임스탬프로 명확히 기각**했다 (t=713 vs t=886). 유력했던
  가설을 데이터로 버린 것은 낭비가 아니라 진전이다.
- 회전 가능 조건을 **부등식으로 정량화**해서, 이후 판단이 감이 아니라 계산이 됐다.
- 침대 목표 좌표를 월드 파일에서 **역산으로 검산**해 기존 표의 오류를 잡았다.
- rosbag을 켜둔 덕에 실패한 주행도 데이터로 남았다.

---

## 6. 내일 여기서 시작

### STEP A — 음성 측정 재검증 (5분, 최우선)

§2-3의 결론은 "아무것도 안 나왔다"에 의존한다. 이 VM은 DDS/SHM 오류 이력이
있고 `ros2` CLI가 죽은 적도 있어, **음성 결과 자체를 의심해야 한다.**

```bash
ros2 topic info /cmd_vel --verbose            # 발행자 수 확인
ros2 topic info /cmd_vel_smoothed --verbose   # 발행자 2개면 §4-2 확정
ros2 node list | grep -E "controller|smoother|collision"
```

- `/cmd_vel` 발행자가 0이면 → controller_server가 발행 자체를 안 함
- 발행자는 있는데 `hz`가 안 잡혔으면 → **측정 실패였고 §2-3 결론은 무효**

### STEP B — velocity_smoother 검증 (5분)

```bash
ros2 param get /velocity_smoother max_velocity
ros2 param get /velocity_smoother feedback
```

`max_velocity`의 두 번째 값(y)이 `0.0`이면 횡이동이 죽어 있는 것이 확정된다.

**대응 (둘 중 택1, 한 번에 하나만)**
1. `nav2_care.yaml`에 `velocity_smoother` 섹션 추가,
   `max_velocity: [0.3, 0.3, 1.0]`, `min_velocity: [0.0, -0.3, -1.0]`
2. `velocity_smoother`를 아예 내리고 Collision Monitor만 남긴다
   (체인 단순화. `bringup_launch.py`를 직접 include하는 구조를 손봐야 함)

### STEP C — 목표 재시도

STEP A/B에서 뭔가 고쳤으면 병실 진입 목표만 다시 보낸다.
**변경은 하나씩, 결과는 3회 반복해서 성공률로 판단한다.**

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "map"}, pose: {position: {x: 9.18, y: 0.0, z: 0.0},
 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

### 아직 안 건드린 후보 (STEP A/B로 안 풀릴 때)

- `controller_frequency` 20 → 10 + `model_dt` 0.05 → 0.1 (**반드시 짝으로**).
  파일 주석에 근거 기재됨. MPPI 부하 절반.
- `vx_min: 0.0`(후진 차단)이 좁은 구간에서 해 공간을 과도하게 줄이는지.
  진단용으로 `-0.1`로 잠깐 풀어 확인.
- SLAM 지도(`hospital_4bed_map.yaml`)의 도어웨이 부근 셀 상태.
  `global_costmap`의 `track_unknown_space: true`와 상호작용 가능성.

---

## 7. 영상 촬영 계획 (주행 확정 후)

요구사항은 **RViz 화면이 아니라 직접 만든 3D 병동 월드 안을 주행하는 그림**이다.
한이음 제출본 편집에 쓸 품질이어야 한다.

VM에 GPU가 없어 `gzclient` + 화면녹화를 주행과 동시에 돌리면 §2-1의 문제가
그대로 재현된다. 따라서 **분리**한다.

1. **주행** — headless(`gui:=false`)로 성공 확인 + Gazebo 상태 로그 기록
2. **촬영** — Gazebo GUI에서 그 로그를 재생하며 화면 녹화
   (Nav2·물리 연산이 없으므로 CPU 전량이 렌더링에 할당됨)

`gz log --help`로 기록/재생 옵션 확인 후 절차 확정할 것.

---

## 8. 오늘 변경된 파일

| 파일 | 변경 |
|---|---|
| `nav2_care.yaml` | AMCL `max_beams` 120→60, `update_min_a/d` 0.15→0.2/0.25 (회귀 되돌림) |
| `nav2_care.yaml` | local/global costmap `footprint` 0.499×0.451 → 0.34×0.30 |
| `hanium_시뮬_이어서하기_0803.md` | 침대 목표 좌표표 갱신 + 산출 근거 기재 |

`alpha1~5: 0.05`는 유지 (CPU 비용 없음, 효과는 미검증).
