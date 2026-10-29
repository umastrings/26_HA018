#!/usr/bin/env python3
"""
calibrate_drive.py — 구동계 캘리브레이션 (ROS 없이 UNO 직결)

  로드맵 §2-1 의 실측 항목을 순서대로 뽑아낸다.
  결과는 calib_result.json 에 누적 저장되고, 마지막에 붙여넣을 파라미터를 출력한다.

  ★ 왜 ROS 를 안 쓰나
    serial_bridge_node 는 항상 FF+PI 폐루프를 돌린다. 그런데 이 스크립트가 재려는
    건 "PWM 을 넣으면 몇 rad/s 로 도는가"(= 폐루프의 재료)다. 폐루프가 돌면
    그걸 못 잰다. 그래서 UNO 와 직접 통신하는 개루프 도구가 맞다.
    부수효과로 colcon build 없이도 돌아간다.

  ★ 전제
    · 바퀴를 공중에 띄운 상태 (받침대/박스). 무부하 기준값을 먼저 잡는다.
    · UNO 펌웨어 업로드 완료, ROS 노드는 모두 종료 (포트 동시 사용 불가)
    · pyserial 필요:  pip3 install pyserial

  사용법
    python3 tools/calibrate_drive.py enc        # ① 감속비 + enc_sign  (손으로 굴림)
    python3 tools/calibrate_drive.py motor      # ② wheel_sign         (저속 구동)
    python3 tools/calibrate_drive.py step       # ③ ff_gain + deadband (계단 입력)
    python3 tools/calibrate_drive.py watchdog   # ④ 300ms 워치독 확인
    python3 tools/calibrate_drive.py report     # 결과 요약 + 파라미터 출력

    공통 옵션:  --port /dev/ttyCH341USB0  --baud 115200
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(
    os.path.join(HERE, '..', 'src', 'smart_upcare_control')))

try:
    import serial
except ImportError:
    sys.exit("pyserial 이 없습니다:  pip3 install pyserial")

# 프로토콜은 패키지 원본을 그대로 씀 (스펙 이중화 방지)
from smart_upcare_control.serial_protocol import (      # noqa: E402
    FrameScanner, PacketStats, build_motor_downlink,
)
from smart_upcare_control.mecanum_kinematics import (   # noqa: E402
    CPR, RAD_PER_PULSE,
)

RESULT_PATH = os.path.join(HERE, 'calib_result.json')
WHEELS = ['M1(앞좌/FL)', 'M2(뒤좌/RL)', 'M3(뒤우/RR)', 'M4(앞우/FR)']

DEFAULT_PORT = '/dev/ttyCH341USB0'   # CH340 클론 UNO (구동팀 회신 §6-1)

# 타이밍 — 테스트에서 줄여 쓸 수 있게 모듈 레벨로 뺌
RESET_WAIT_S = 2.0      # UNO 는 포트 열 때 DTR 토글로 재부팅된다
SETTLE_S = 0.5          # 계단 인가 후 정상상태 도달 대기
MEASURE_S = 1.0         # 측정 구간
COOL_S = 0.3            # 다음 단계 전 정지 구간
STEP_LEVELS = [20, 30, 40, 60, 80, 100, 130, 160, 200, 255]

# ★ 8/13 실차 — 워치독 관측창.
#   펌웨어 stopMotors() 는 RELEASE(코스팅)라 명령이 끊겨도 바퀴가 즉시 서지 않는다.
#   1.5s 단일 구간으로 보면 관성 감속이 "정지 실패"로 잘못 읽힌다.
WD_IDLE_S = 2.5         # 명령 끊고 관측하는 총 시간
WD_BIN_S = 0.25         # 감속 곡선을 나눠 보는 구간 폭
WD_STOP_TH = 0.2        # 이 각속도 미만이면 정지로 본다 [rad/s]


# ══════════════════════════════════════════════════════════════════
# 시리얼 래퍼
# ══════════════════════════════════════════════════════════════════
class Uno:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except (serial.SerialException, OSError) as e:
            sys.exit(f"포트를 못 열었습니다: {port}\n  {e}\n\n"
                     f"확인할 것:\n"
                     f"  · ls -l /dev/serial/by-id/   (실제 경로 확인)\n"
                     f"  · ROS 노드나 miniterm 이 포트를 잡고 있지 않은지\n"
                     f"  · 권한:  sudo usermod -aG dialout $USER  (재로그인 필요)")
        self.stats = PacketStats()
        self.scan = FrameScanner(self.stats)
        self.ticks = None
        time.sleep(RESET_WAIT_S)   # UNO 리셋 대기 (DTR 토글로 재부팅됨)
        self.ser.reset_input_buffer()

    def poll(self):
        """수신 처리. 새 엔코더 값이 있으면 (t, ticks) 반환, 없으면 None"""
        try:
            n = self.ser.in_waiting
            chunk = self.ser.read(n if n else 1)
        except (serial.SerialException, OSError) as e:
            sys.exit(f"시리얼 읽기 오류: {e}")
        if not chunk:
            return None
        out = None
        for pkt in self.scan.feed(chunk):
            if pkt['type'] == 'E':
                self.ticks = pkt['ticks']
                out = (time.time(), list(pkt['ticks']))
        return out

    def wait_ticks(self, timeout=3.0):
        """엔코더 패킷 하나를 기다린다"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            r = self.poll()
            if r:
                return r
        return None

    def drive(self, pwm4):
        self.ser.write(build_motor_downlink(*pwm4))

    def stop(self):
        for _ in range(5):
            try:
                self.drive([0, 0, 0, 0])
            except Exception:
                pass
            time.sleep(0.02)

    def close(self):
        self.stop()
        try:
            self.ser.close()
        except Exception:
            pass

    def sample(self, duration, pwm4=None, hold_hz=50.0):
        """
        duration 초 동안 pwm4 를 유지하며 (t, ticks) 를 모은다.
        pwm4=None 이면 아무 명령도 안 보냄(손으로 굴리는 모드).
        """
        samples = []
        t0 = time.time()
        next_cmd = 0.0
        while time.time() - t0 < duration:
            now = time.time()
            if pwm4 is not None and now >= next_cmd:
                self.drive(pwm4)          # 펌웨어 300ms 워치독 만족용 연속 발행
                next_cmd = now + 1.0 / hold_hz
            r = self.poll()
            if r:
                samples.append(r)
            time.sleep(0.002)
        return samples


# ══════════════════════════════════════════════════════════════════
# 계산
# ══════════════════════════════════════════════════════════════════
def omega_from_samples(samples, wheel_idx):
    """
    (t, ticks) 목록 -> 해당 바퀴 각속도 [rad/s]
    양 끝점 차분 대신 최소자승 기울기를 쓴다 — 시리얼 지터에 훨씬 덜 흔들림.
    """
    if len(samples) < 3:
        return 0.0
    t0 = samples[0][0]
    xs = [s[0] - t0 for s in samples]
    ys = [s[1][wheel_idx] for s in samples]
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    den = sum((x - mx) ** 2 for x in xs)
    if den == 0:
        return 0.0
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den   # pulse/s
    return slope * RAD_PER_PULSE


def load_result():
    if os.path.exists(RESULT_PATH):
        with open(RESULT_PATH, encoding='utf-8') as f:
            return json.load(f)
    return {}


def save_result(d):
    with open(RESULT_PATH, 'w', encoding='utf-8') as f:
        json.dump(d, f, ensure_ascii=False, indent=2)
    print(f"\n저장됨: {RESULT_PATH}")


def hr(title=''):
    print('\n' + '─' * 66)
    if title:
        print(title)
        print('─' * 66)


# ══════════════════════════════════════════════════════════════════
# ① 감속비 + enc_sign  (손으로 굴림)
# ══════════════════════════════════════════════════════════════════
def mode_enc(uno, res):
    hr('① 감속비 실측 + enc_sign  — 손으로 굴리는 단계')
    print("""
바퀴를 하나씩 손으로 굴립니다. 모터에는 전원을 넣지 않습니다.

  · '전진 방향' = 로봇이 앞으로 갈 때 그 바퀴가 도는 방향입니다.
    (메카넘 전진은 네 바퀴가 모두 같은 방향으로 돕니다)
  · 정확히 한 바퀴(360°) 돌리세요. 밸브/마킹을 기준점으로 잡으면 편합니다.
  · 카운트가 4320 근처면 감속비 90:1, 1440 근처면 30:1 입니다.
""")
    if uno.wait_ticks() is None:
        sys.exit("엔코더 패킷 <E,..> 이 안 옵니다. 펌웨어 업로드와 배선을 확인하세요.")

    enc_sign = [0, 0, 0, 0]
    counts = [0, 0, 0, 0]
    for i, name in enumerate(WHEELS):
        hr(f'  [{i+1}/4] {name}')
        input(f"    {name} 를 잡고 준비되면 Enter → 그 다음 전진 방향으로 한 바퀴")
        _, before = uno.wait_ticks()
        print("    지금 굴리세요. 다 돌렸으면 Enter")
        # 굴리는 동안 계속 수신
        import threading
        done = threading.Event()
        threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
        while not done.is_set():
            uno.poll()
            time.sleep(0.005)
        after = uno.ticks
        d = after[i] - before[i]
        counts[i] = d
        enc_sign[i] = 1 if d > 0 else (-1 if d < 0 else 0)

        mag = abs(d)
        if mag < 100:
            verdict = "⚠️  변화가 너무 작음 — 엔코더 배선이나 굴린 양 확인"
        elif abs(mag - 4320) < abs(mag - 1440):
            verdict = f"90:1 로 보임 (4320 기준, 오차 {mag - 4320:+d})"
        else:
            verdict = f"⚠️  30:1 로 보임 (1440 기준, 오차 {mag - 1440:+d}) — 감속비 재검토 필요"
        print(f"    → 카운트 {d:+d}   enc_sign = {enc_sign[i]:+d}   {verdict}")

        # 다른 바퀴가 같이 돌았는지 (배선 교차 검출)
        cross = [j for j in range(4)
                 if j != i and abs(after[j] - before[j]) > 50]
        if cross:
            print(f"    ⚠️  {', '.join(WHEELS[j] for j in cross)} 도 같이 변했습니다. "
                  f"엔코더 배선이 교차됐을 수 있습니다.")

    hr('① 결과')
    print(f"  enc_sign : {enc_sign}")
    print(f"  카운트   : {counts}   (한 바퀴 기준)")
    avg = sum(abs(c) for c in counts) / 4
    print(f"  평균 {avg:.0f} pulse/rev  →  코드값 CPR={CPR}")
    if abs(avg - CPR) > CPR * 0.1:
        print(f"  ⚠️  코드의 CPR({CPR})과 10% 넘게 차이납니다. "
              f"mecanum_kinematics.py 의 GEAR_RATIO 를 재확인하세요.")
    res['enc_sign'] = enc_sign
    res['counts_per_rev_measured'] = counts
    save_result(res)


# ══════════════════════════════════════════════════════════════════
# ② wheel_sign  (저속 구동)
# ══════════════════════════════════════════════════════════════════
def mode_motor(uno, res):
    hr('② wheel_sign 판정 — 바퀴별 저속 구동')
    if 'enc_sign' not in res:
        sys.exit("먼저 `enc` 모드를 실행하세요. enc_sign 이 있어야 판정할 수 있습니다.")
    enc_sign = res['enc_sign']
    print(f"""
enc_sign = {enc_sign} 를 기준으로 판정합니다.

  각 바퀴에 +PWM 을 잠깐 넣고, 그 바퀴가 '전진 방향'으로 도는지 봅니다.
    · 전진 방향으로 돌면  wheel_sign = +1
    · 반대로 돌면         wheel_sign = -1

★ 바퀴가 공중에 떠 있는지 다시 확인하세요. 한 바퀴씩만 돕니다.
""")
    if input("  준비됐으면 y 입력: ").strip().lower() != 'y':
        sys.exit("중단")

    PWM = 120
    DUR = MEASURE_S + 0.2
    wheel_sign = [0, 0, 0, 0]
    for i, name in enumerate(WHEELS):
        print(f"\n  [{i+1}/4] {name} — PWM +{PWM} 로 {DUR}초")
        cmd = [0, 0, 0, 0]
        cmd[i] = PWM
        uno.sample(SETTLE_S, pwm4=cmd)                # 기동 과도구간 버림
        samples = uno.sample(DUR, pwm4=cmd)
        uno.stop()
        time.sleep(COOL_S)

        raw = omega_from_samples(samples, i)
        if abs(raw) < 0.2:
            print(f"    ⚠️  거의 안 돌았습니다 ({raw:+.2f} rad/s). "
                  f"모터 배선·전원·PWM 데드밴드 확인")
            wheel_sign[i] = 1
            continue
        # 실제 kinematic 방향 = raw 부호 / enc_sign
        kin_dir = (1 if raw > 0 else -1) * enc_sign[i]
        wheel_sign[i] = kin_dir      # +PWM 이 전진이면 +1, 아니면 -1
        print(f"    raw {raw:+.2f} rad/s  ·  enc_sign {enc_sign[i]:+d}"
              f"  →  wheel_sign = {wheel_sign[i]:+d}"
              f"  ({'전진' if kin_dir > 0 else '후진'} 방향)")

        # 다른 바퀴가 같이 돌았는지
        for j in range(4):
            if j != i and abs(omega_from_samples(samples, j)) > 0.3:
                print(f"    ⚠️  {WHEELS[j]} 도 같이 돌았습니다. 모터 배선 교차 의심")

    hr('② 결과')
    print(f"  wheel_sign : {wheel_sign}")
    print(f"  enc_sign   : {enc_sign}")
    mixed = [i for i in range(4) if wheel_sign[i] != enc_sign[i]]
    if mixed:
        print(f"\n  ★ {', '.join(WHEELS[i] for i in mixed)} 는 wheel_sign ≠ enc_sign 입니다.")
        print(f"    모터 극성과 엔코더 극성이 따로 논다는 뜻이고, 두 파라미터를")
        print(f"    분리해두지 않았으면 이 바퀴가 폐루프에서 폭주했을 조합입니다.")
    else:
        print("\n  두 부호가 일치합니다. (enc_sign 분리 없이도 동작했을 조합)")
    res['wheel_sign'] = wheel_sign
    save_result(res)


# ══════════════════════════════════════════════════════════════════
# ③ ff_gain + deadband  (계단 입력)
# ══════════════════════════════════════════════════════════════════
def mode_step(uno, res):
    hr('③ PWM ↔ rad/s 맵 — 계단 입력')
    wheel_sign = res.get('wheel_sign', [1, 1, 1, 1])
    enc_sign = res.get('enc_sign', [1, 1, 1, 1])
    print("""
PWM 을 단계별로 올리며 각 바퀴의 정상상태 각속도를 잽니다.
여기서 ff_gain(각속도→PWM 환산)과 deadband(움직이기 시작하는 최소 PWM)가 나옵니다.

★ 네 바퀴가 동시에 돕니다. 공중에 떠 있는지 확인하세요.
""")
    if input("  준비됐으면 y 입력: ").strip().lower() != 'y':
        sys.exit("중단")

    table = []
    for pwm in STEP_LEVELS:
        # kinematic 전진 방향으로 통일해서 인가 (부호는 wheel_sign 적용)
        cmd = [pwm * s for s in wheel_sign]
        uno.sample(SETTLE_S, pwm4=cmd)                # 정상상태 도달 대기
        samples = uno.sample(MEASURE_S, pwm4=cmd)
        uno.stop()
        time.sleep(COOL_S)
        # 측정값을 kinematic 공간으로 환산: raw / enc_sign
        w = [omega_from_samples(samples, i) / enc_sign[i] for i in range(4)]
        table.append((pwm, w))
        print(f"  PWM {pwm:3d}  →  " +
              "  ".join(f"{x:+6.2f}" for x in w) + "  rad/s")

    hr('③ 결과')
    # deadband: 평균 각속도가 0.3 rad/s 를 처음 넘는 PWM 의 한 단계 아래
    deadband = 0
    for pwm, w in table:
        if sum(abs(x) for x in w) / 4 > 0.3:
            deadband = pwm
            break
    print(f"  deadband_pwm ≈ {deadband}  (이 값 미만에서는 거의 안 돎)")

    # ff_gain: 선형구간(deadband 초과 & 포화 전) 최소자승 기울기 PWM/omega
    ff_per_wheel = []
    for i in range(4):
        pts = [(w[i], pwm) for pwm, w in table
               if abs(w[i]) > 0.5 and pwm <= 200]
        if len(pts) < 3:
            ff_per_wheel.append(None)
            continue
        sx = sum(abs(p[0]) for p in pts)
        sy = sum(p[1] for p in pts)
        sxx = sum(p[0] ** 2 for p in pts)
        sxy = sum(abs(p[0]) * p[1] for p in pts)
        n = len(pts)
        den = n * sxx - sx * sx
        ff_per_wheel.append((n * sxy - sx * sy) / den if den else None)

    valid = [f for f in ff_per_wheel if f]
    ff_gain = sum(valid) / len(valid) if valid else None
    for i, f in enumerate(ff_per_wheel):
        print(f"  {WHEELS[i]:14s} ff_gain = "
              + (f"{f:6.1f}" if f else "  측정 실패"))
    if ff_gain:
        print(f"\n  ff_gain(평균) ≈ {ff_gain:.1f}   (코드 기본값 40.6)")
        spread = (max(valid) - min(valid)) / ff_gain * 100
        if spread > 20:
            print(f"  ⚠️  바퀴별 편차 {spread:.0f}% — 20% 넘습니다. "
                  f"모터 개체차가 크면 횡이동 시 사선으로 밀립니다.")

    # 최대 각속도 (정격 확인)
    max_w = max(abs(x) for _, w in table for x in w)
    print(f"  실측 최대 각속도 ≈ {max_w:.2f} rad/s  "
          f"(코드 max_wheel_omega 기본값 6.283)")

    res['deadband_pwm'] = deadband
    res['ff_gain'] = round(ff_gain, 1) if ff_gain else None
    res['ff_per_wheel'] = [round(f, 1) if f else None for f in ff_per_wheel]
    res['max_wheel_omega_measured'] = round(max_w, 2)
    res['step_table'] = [(pwm, [round(x, 3) for x in w]) for pwm, w in table]
    save_result(res)


# ══════════════════════════════════════════════════════════════════
# ④ 워치독
# ══════════════════════════════════════════════════════════════════
def mode_watchdog(uno, res):
    hr('④ 펌웨어 300ms 워치독 확인')
    print("""
바퀴를 돌린 뒤 명령을 끊고, 실제로 멈추는지 봅니다.
펌웨어의 CMD_TIMEOUT_MS(300) 가 동작하는지 확인하는 단계입니다.

  ※ 펌웨어 stopMotors() 는 RELEASE(양쪽 핀 LOW = 코스팅)라
     워치독이 제때 걸려도 바퀴는 관성으로 더 돕니다.
     그래서 "멈췄나"가 아니라 "언제부터 감속했나"를 봐야 합니다.
       · 계속 8 rad/s 근처   -> 워치독 자체가 안 걸린 것 (진짜 문제)
       · 단조 감속 후 0      -> 워치독 정상, 코스팅 거리만 남는 것

★ 바퀴 공중 확인.
""")
    if input("  준비됐으면 y 입력: ").strip().lower() != 'y':
        sys.exit("중단")

    cmd = [120] * 4
    uno.sample(0.5, pwm4=cmd)
    moving = uno.sample(0.8, pwm4=cmd)
    w_before = sum(abs(omega_from_samples(moving, i)) for i in range(4)) / 4
    print(f"  구동 중 평균 각속도 : {w_before:.2f} rad/s")

    print(f"  명령 중단 — {WD_IDLE_S:.1f}초 관측 (아무것도 안 보냄)\n")
    idle = uno.sample(WD_IDLE_S, pwm4=None)          # 명령 끊김
    if len(idle) < 6:
        sys.exit("  엔코더 수신이 끊겨 판정할 수 없습니다.")

    # 구간별 각속도 — 감속 곡선을 그대로 보여준다
    t0 = idle[0][0]
    print("    경과        평균 각속도")
    stop_at = None
    for b in range(int(WD_IDLE_S / WD_BIN_S)):
        lo, hi = b * WD_BIN_S, (b + 1) * WD_BIN_S
        seg = [s for s in idle if lo <= s[0] - t0 < hi]
        if len(seg) < 3:
            continue
        w = sum(abs(omega_from_samples(seg, i)) for i in range(4)) / 4
        flag = ''
        if w < WD_STOP_TH and stop_at is None:
            stop_at = hi
            flag = '  <- 정지'
        print(f"    ~{hi:4.2f}s      {w:6.2f} rad/s{flag}")

    # 판정은 마지막 0.5초 — 관성 감속이 끝났을 시점
    tail = [s for s in idle if s[0] - t0 >= WD_IDLE_S - 0.5]
    w_after = (sum(abs(omega_from_samples(tail, i)) for i in range(4)) / 4
               if len(tail) >= 3 else 0.0)
    print(f"\n  마지막 0.5초 각속도 : {w_after:.2f} rad/s")

    ok = w_after < WD_STOP_TH
    if ok:
        coast = f"{stop_at:.2f}초" if stop_at else f"{WD_IDLE_S:.1f}초 이내"
        print(f"  ✅ 워치독 정상 — 명령 끊긴 뒤 {coast} 만에 정지 (코스팅 포함)")
    else:
        print("  ❌ 끝까지 안 멈춤 — 펌웨어 CMD_TIMEOUT_MS 확인 필요")
    res['watchdog_ok'] = ok
    res['watchdog_coast_s'] = stop_at
    res['watchdog_omega_before'] = round(w_before, 3)
    save_result(res)


# ══════════════════════════════════════════════════════════════════
# ⑤ 리포트
# ══════════════════════════════════════════════════════════════════
def mode_report(res):
    hr('캘리브레이션 결과 요약')
    if not res:
        sys.exit(f"결과 파일이 없습니다: {RESULT_PATH}\n먼저 enc / motor / step 을 실행하세요.")

    done = [k for k in ('enc_sign', 'wheel_sign', 'ff_gain', 'watchdog_ok') if k in res]
    print(f"  완료: {', '.join(done) if done else '없음'}")

    counts = res.get('counts_per_rev_measured')
    if counts:
        avg = sum(abs(c) for c in counts) / 4
        gear = 90 if abs(avg - 4320) < abs(avg - 1440) else 30
        print(f"\n  감속비 : {gear}:1  (실측 평균 {avg:.0f} pulse/rev)")
        if gear != 90:
            print("  ⚠️  코드는 90:1(CPR 4320) 기준입니다. "
                  "mecanum_kinematics.py 의 GEAR_RATIO 를 고쳐야 합니다.")

    hr('robot_bringup / drive_test 에 넣을 파라미터')
    print("""ros2 launch smart_upcare_control drive_test.launch.py \\""")
    lines = []
    if 'wheel_sign' in res:
        lines.append(f"    --ros-args -p wheel_sign:={res['wheel_sign']}")
    if 'enc_sign' in res:
        lines.append(f"    -p enc_sign:={res['enc_sign']}")
    if res.get('ff_gain'):
        lines.append(f"    -p ff_gain:={res['ff_gain']}")
    if 'deadband_pwm' in res:
        lines.append(f"    -p deadband_pwm:={res['deadband_pwm']}")
    if res.get('max_wheel_omega_measured'):
        lines.append(f"    -p max_wheel_omega:={res['max_wheel_omega_measured']}")
    print(' \\\n'.join(lines) if lines else '    (측정된 값 없음)')

    hr('serial_bridge_node.py 기본값으로 굳힐 값')
    for k in ('wheel_sign', 'enc_sign', 'ff_gain', 'deadband_pwm'):
        if k in res and res[k] is not None:
            print(f"    self.declare_parameter('{k}', {res[k]})")

    if 'wheel_sign' in res and 'enc_sign' in res:
        mixed = [i for i in range(4) if res['wheel_sign'][i] != res['enc_sign'][i]]
        if mixed:
            hr('★ 구동팀에 전달할 내용')
            print(f"  {', '.join(WHEELS[i] for i in mixed)} 는 모터 극성과 엔코더 극성이 다릅니다.")
            print("  펌웨어에 ENC_SIGN[4] 를 되살려 UNO 가 CCW=+ 로 통일해 보내주면")
            print("  젯슨 enc_sign 을 [1,1,1,1] 로 둘 수 있어 구조가 깔끔해집니다.")


# ══════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description='구동계 캘리브레이션 (UNO 직결)')
    ap.add_argument('mode', choices=['enc', 'motor', 'step', 'watchdog', 'report'])
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=115200)
    a = ap.parse_args()

    res = load_result()
    if a.mode == 'report':
        mode_report(res)
        return

    print(f"포트 {a.port} @ {a.baud}  — UNO 리셋 대기 2초...")
    uno = Uno(a.port, a.baud)
    try:
        {'enc': mode_enc, 'motor': mode_motor,
         'step': mode_step, 'watchdog': mode_watchdog}[a.mode](uno, res)
    except KeyboardInterrupt:
        print("\n중단됨 — 모터 정지")
    finally:
        uno.close()
        print(f"\n패킷 통계: {uno.stats}")


if __name__ == '__main__':
    main()
