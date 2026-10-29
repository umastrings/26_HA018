#!/usr/bin/env python3
"""
test_calibrate_drive.py — calibrate_drive.py 를 가짜 UNO 상대로 검증

  pty(가상 시리얼 포트) 위에 UNO 펌웨어를 흉내내는 스레드를 띄우고,
  calibrate_drive 의 각 모드를 실제로 돌려서 **미리 아는 정답을 되찾는지** 본다.

  가짜 UNO 가 흉내내는 것
    · <M,..> 파싱 + <E,..> 25Hz 스트리밍 (실물 펌웨어와 동일)
    · 300ms 워치독
    · PWM 데드밴드 + 선형 속도 응답
    · ★ 모터 극성과 엔코더 극성을 따로 설정 — 실차 조건(우측 2바퀴 배선 반대) 재현

  실행:  python3 tools/test_calibrate_drive.py
"""
import os
import pty
import sys
import threading
import time
import json
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.abspath(
    os.path.join(HERE, '..', 'src', 'smart_upcare_control')))

import calibrate_drive as cal                                    # noqa: E402
from smart_upcare_control.mecanum_kinematics import RAD_PER_PULSE  # noqa: E402

# ── 정답 (가짜 UNO 의 물리 설정) ──────────────────────────────────
# motor_sign : +PWM 을 넣으면 그 바퀴가 kinematic 어느 방향으로 도는가
# enc_true   : 바퀴가 kinematic 전진으로 돌 때 엔코더가 증가(+1)/감소(-1)
TRUE_MOTOR_SIGN = [1, 1, -1, -1]     # 구동팀 회신 §6-2 — 우측 2바퀴 배선 반대
TRUE_ENC_SIGN = [1, 1, 1, 1]         # 엔코더는 손 안 댐 → 정상 극성
TRUE_DEADBAND = 25                   # 이 PWM 미만은 안 돎
TRUE_K = 6.0 / (255 - TRUE_DEADBAND)  # PWM 255 에서 6.0 rad/s
TRUE_FF = 1.0 / TRUE_K               # 스크립트가 되찾아야 할 ff_gain ≈ 38.3

UPLINK_HZ = 25.0
WATCHDOG_S = 0.3


class FakeUno(threading.Thread):
    def __init__(self, fd):
        super().__init__(daemon=True)
        self.fd = fd
        self.running = True
        self.pos = [0.0, 0.0, 0.0, 0.0]      # 누적 엔코더 (float 로 들고 있다 int 로 송신)
        self.pwm = [0, 0, 0, 0]
        self.last_cmd = 0.0
        self.hand = [0.0, 0.0, 0.0, 0.0]     # 손으로 굴린 양 (kinematic rad)
        self.buf = b''
        self.lock = threading.Lock()

    def turn_by_hand(self, wheel, kin_rad):
        """바퀴를 손으로 kinematic 방향 kin_rad 만큼 굴린다"""
        with self.lock:
            self.pos[wheel] += TRUE_ENC_SIGN[wheel] * kin_rad / RAD_PER_PULSE

    def _parse(self, data):
        self.buf += data
        while b'<' in self.buf and b'>' in self.buf:
            s = self.buf.index(b'<')
            e = self.buf.index(b'>', s)
            body = self.buf[s + 1:e].decode('ascii', 'ignore')
            self.buf = self.buf[e + 1:]
            parts = body.split(',')
            if parts[0] == 'M' and len(parts) == 5:
                try:
                    with self.lock:
                        self.pwm = [int(x) for x in parts[1:]]
                        self.last_cmd = time.time()
                except ValueError:
                    pass

    def run(self):
        os.set_blocking(self.fd, False)
        last_tick = time.time()
        last_up = 0.0
        while self.running:
            try:
                data = os.read(self.fd, 4096)
                if data:
                    self._parse(data)
            except BlockingIOError:
                pass
            except OSError:
                break

            now = time.time()
            dt = now - last_tick
            last_tick = now

            with self.lock:
                pwm = list(self.pwm)
                if now - self.last_cmd > WATCHDOG_S:      # 펌웨어 워치독
                    pwm = [0, 0, 0, 0]
                for i in range(4):
                    a = abs(pwm[i])
                    if a <= TRUE_DEADBAND:
                        kin = 0.0
                    else:
                        mag = TRUE_K * (a - TRUE_DEADBAND)
                        # +PWM -> motor_sign 방향으로 kinematic 회전
                        kin = mag * (1 if pwm[i] > 0 else -1) * TRUE_MOTOR_SIGN[i]
                    # 엔코더는 kinematic 회전을 enc_sign 극성으로 읽는다
                    self.pos[i] += TRUE_ENC_SIGN[i] * kin * dt / RAD_PER_PULSE

                if now - last_up >= 1.0 / UPLINK_HZ:
                    last_up = now
                    line = "<E,%d,%d,%d,%d>\r\n" % tuple(int(p) for p in self.pos)
                    try:
                        os.write(self.fd, line.encode())
                    except OSError:
                        break
            time.sleep(0.001)


PASS, FAIL = [], []


def check(name, cond, detail=''):
    (PASS if cond else FAIL).append(name)
    print(f"  {'✅' if cond else '❌'} {name}" + (f"  — {detail}" if detail else ''))


class Quiet:
    """모드 함수들이 뿜는 출력을 숨긴다 (검증 결과만 보이게)"""
    def __enter__(self):
        self._o = sys.stdout
        sys.stdout = open(os.devnull, 'w')
        return self

    def __exit__(self, *a):
        sys.stdout.close()
        sys.stdout = self._o


def main():
    # 테스트용으로 타이밍 축소
    cal.RESET_WAIT_S = 0.2
    cal.SETTLE_S = 0.15
    cal.MEASURE_S = 0.6
    cal.COOL_S = 0.1
    cal.STEP_LEVELS = [20, 40, 80, 120, 180, 255]
    # 마운트 폴더엔 파일을 남기지 않는다 (삭제 권한이 없을 수 있음)
    cal.RESULT_PATH = os.path.join(tempfile.gettempdir(), 'calib_result_TEST.json')
    if os.path.exists(cal.RESULT_PATH):
        os.remove(cal.RESULT_PATH)

    master, slave = pty.openpty()
    port = os.ttyname(slave)
    fake = FakeUno(master)
    fake.start()
    print(f"가짜 UNO 기동: {port}")
    print(f"정답 → motor_sign={TRUE_MOTOR_SIGN}  enc_sign={TRUE_ENC_SIGN}  "
          f"deadband={TRUE_DEADBAND}  ff_gain≈{TRUE_FF:.1f}")

    uno = cal.Uno(port, 115200)
    res = {}

    # ── ① enc: 손으로 굴리는 모드 — input() 을 가로채 자동화 ──────
    print("\n① enc — 감속비 + enc_sign")
    seq = iter(range(100))

    def fake_input(prompt=''):
        n = next(seq)
        # 짝수번째 호출(=굴리기 시작 신호) 뒤에 실제로 한 바퀴 굴린다
        if n % 2 == 1:
            wheel = n // 2
            fake.turn_by_hand(wheel, 2 * 3.141592653589793)   # 정확히 1회전
            time.sleep(0.15)      # 업링크 몇 개 지나가게
        return ''

    import builtins
    orig_input = builtins.input
    builtins.input = fake_input
    try:
        with Quiet():
            cal.mode_enc(uno, res)
    finally:
        builtins.input = orig_input

    check('enc_sign 복원', res['enc_sign'] == TRUE_ENC_SIGN,
          f"{res['enc_sign']} (정답 {TRUE_ENC_SIGN})")
    cnt = res['counts_per_rev_measured']
    check('한 바퀴 카운트가 CPR(4320) 근처',
          all(abs(abs(c) - 4320) < 200 for c in cnt), str(cnt))

    # ── ② motor: wheel_sign ────────────────────────────────────
    print("\n② motor — wheel_sign")
    builtins.input = lambda *a: 'y'
    try:
        with Quiet():
            cal.mode_motor(uno, res)
    finally:
        builtins.input = orig_input
    check('wheel_sign 복원 (우측 2바퀴 -1 검출)',
          res['wheel_sign'] == TRUE_MOTOR_SIGN,
          f"{res['wheel_sign']} (정답 {TRUE_MOTOR_SIGN})")

    # ── ③ step: ff_gain + deadband ─────────────────────────────
    print("\n③ step — ff_gain + deadband")
    builtins.input = lambda *a: 'y'
    try:
        with Quiet():
            cal.mode_step(uno, res)
    finally:
        builtins.input = orig_input

    check('deadband 검출 (정답 25, 인접 단계 허용)',
          res['deadband_pwm'] in (20, 40), str(res['deadband_pwm']))
    ff = res['ff_gain']
    check('ff_gain 오차 15% 이내',
          ff and abs(ff - TRUE_FF) / TRUE_FF < 0.15,
          f"{ff} (정답 {TRUE_FF:.1f})")
    # 계단표의 부호 — wheel_sign 적용 후엔 전 바퀴가 같은 부호(+)여야 함
    top = res['step_table'][-1][1]
    check('계단표에서 전 바퀴 같은 방향(전진)',
          all(x > 0 for x in top), str(top))
    check('최대 각속도 실측', abs(res['max_wheel_omega_measured'] - 6.0) < 0.6,
          str(res['max_wheel_omega_measured']))

    # ── ④ watchdog ─────────────────────────────────────────────
    print("\n④ watchdog — 명령 끊김 시 정지")
    builtins.input = lambda *a: 'y'
    try:
        with Quiet():
            cal.mode_watchdog(uno, res)
    finally:
        builtins.input = orig_input
    check('워치독 정상 판정', res['watchdog_ok'] is True)

    # ── ⑤ report 가 죽지 않는지 ────────────────────────────────
    print("\n⑤ report — 요약 출력")
    try:
        with Quiet():
            cal.mode_report(res)
        check('report 정상 실행', True)
    except Exception as e:
        check('report 정상 실행', False, str(e))

    # ── ⑥ 프로토콜 건전성 ──────────────────────────────────────
    print("\n⑥ 통신 품질")
    check('파싱 오류 없음',
          uno.stats.bad_format == 0 and uno.stats.overflow == 0, str(uno.stats))
    check('패킷 충분히 수신 (>100)', uno.stats.ok > 100, str(uno.stats.ok))

    uno.close()
    fake.running = False
    time.sleep(0.1)
    try:
        os.remove(cal.RESULT_PATH)
    except OSError:
        pass

    print(f"\n{'='*56}\n통과 {len(PASS)} / 실패 {len(FAIL)}")
    if FAIL:
        print('실패:', FAIL)
    sys.exit(1 if FAIL else 0)


if __name__ == '__main__':
    main()
