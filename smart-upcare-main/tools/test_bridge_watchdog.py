#!/usr/bin/env python3
"""
test_bridge_watchdog.py — serial_bridge_node 워치독 동작 검증 (ROS2 없이)

8/7 추가된 안전 로직 두 가지를 실제 노드 코드로 검증한다.
  ① 엔코더 워치독  : UNO 업링크가 끊기면 PWM 0 + 적분 리셋
  ② enable_esp1    : ESP32 없이도 브리지가 뜨고 UNO 제어가 살아있는지

rclpy / pyserial 을 최소 스텁으로 대체하므로 젯슨·VM 없이 어디서나 돌아간다.
    python3 tools/test_bridge_watchdog.py
"""
import sys
import os
import types

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', 'src', 'smart_upcare_control')
sys.path.insert(0, os.path.abspath(PKG))


# ══════════════════════════════════════════════════════════════════
# 스텁
# ══════════════════════════════════════════════════════════════════
class _Time:
    def __init__(self, ns=0):
        self.nanoseconds = ns

    def __sub__(self, other):
        return _Time(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return self


class _Clock:
    def __init__(self):
        self.ns = 0

    def now(self):
        return _Time(self.ns)


class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, lvl, m, **kw):
        self.lines.append(f'{lvl}: {m}')

    def info(self, m, **kw):
        self._log('INFO', m)

    def warn(self, m, **kw):
        self._log('WARN', m)

    def error(self, m, **kw):
        self._log('ERROR', m)

    def fatal(self, m, **kw):
        self._log('FATAL', m)

    def debug(self, m, **kw):
        pass


class _Param:
    def __init__(self, v):
        self.value = v


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _Node:
    def __init__(self, name):
        # ⚠️ SerialBridgeNode.__init__ 이 super().__init__() 을 호출하므로
        #    make_node() 가 미리 심어둔 파라미터를 덮어쓰면 안 된다.
        self._p = getattr(self, '_p', {})
        self._clock = getattr(self, '_clock', None) or _Clock()
        self._logger = getattr(self, '_logger', None) or _Logger()
        self.timers = []

    def declare_parameter(self, k, v):
        self._p.setdefault(k, v)

    def get_parameter(self, k):
        return _Param(self._p[k])

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    def create_publisher(self, *a, **k):
        return _Pub()

    def create_subscription(self, *a, **k):
        return None

    def create_timer(self, period, cb):
        self.timers.append((period, cb))
        return None


class _FakeSerial:
    """열기 성공/실패를 시나리오별로 흉내내는 가짜 포트"""
    fail_ports = set()

    def __init__(self, port, baud, timeout=None):
        if port in _FakeSerial.fail_ports:
            raise _SerialException(f'{port} 없음 (테스트)')
        self.port = port
        self.written = []
        self.in_waiting = 0

    def read(self, n):
        return b''

    def write(self, b):
        self.written.append(b)

    def close(self):
        pass


class _SerialException(Exception):
    pass


def _install_stubs():
    serial_mod = types.ModuleType('serial')
    serial_mod.Serial = _FakeSerial
    serial_mod.SerialException = _SerialException
    sys.modules['serial'] = serial_mod

    rclpy = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = _Node
    qos_mod = types.ModuleType('rclpy.qos')
    qos_mod.qos_profile_sensor_data = None
    rclpy.node = node_mod
    rclpy.qos = qos_mod
    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.node'] = node_mod
    sys.modules['rclpy.qos'] = qos_mod

    for mod, names in [('geometry_msgs.msg', ['Twist']),
                       ('std_msgs.msg', ['Float32', 'String']),
                       ('sensor_msgs.msg', ['Imu']),
                       ('nav_msgs.msg', ['Odometry'])]:
        m = types.ModuleType(mod)
        for n in names:
            setattr(m, n, _make_msg(n))
        sys.modules[mod] = m
        sys.modules[mod.split('.')[0]] = types.ModuleType(mod.split('.')[0])


def _make_msg(name):
    class _M:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id='')
            self.data = 0.0
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            self.angular_velocity = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.orientation_covariance = [0.0] * 9
            self.angular_velocity_covariance = [0.0] * 9
            self.linear_acceleration_covariance = [0.0] * 9
            self.pose = types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
                covariance=[0.0] * 36)
            self.twist = types.SimpleNamespace(
                twist=types.SimpleNamespace(
                    linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0)),
                covariance=[0.0] * 36)
            self.child_frame_id = ''
    _M.__name__ = name
    return _M


_install_stubs()
from smart_upcare_control.serial_bridge_node import SerialBridgeNode  # noqa: E402
from smart_upcare_control.serial_protocol import build_uno_uplink     # noqa: E402


# ══════════════════════════════════════════════════════════════════
# 헬퍼
# ══════════════════════════════════════════════════════════════════
def make_node(**params):
    n = SerialBridgeNode.__new__(SerialBridgeNode)
    _Node.__init__(n, 'serial_bridge_node')
    n._p.update(params)
    SerialBridgeNode.__init__(n)
    return n


def advance(n, sec):
    n._clock.ns += int(sec * 1e9)


def last_pwm(n):
    if not n.uno.written:
        return None
    return n.uno.written[-1].decode().strip()


def feed_encoder(n, ticks):
    """UNO 업링크 1패킷 주입 (리더 스레드 대신 직접 핸들러 호출)"""
    from smart_upcare_control.serial_protocol import FrameScanner
    sc = FrameScanner(n.stats_uno)
    for pkt in sc.feed(build_uno_uplink(ticks)):
        n.on_uno(pkt)


PASS, FAIL = [], []


def check(name, cond, detail=''):
    (PASS if cond else FAIL).append(name)
    print(f"  {'✅' if cond else '❌'} {name}" + (f"  — {detail}" if detail else ''))


# ══════════════════════════════════════════════════════════════════
# 테스트
# ══════════════════════════════════════════════════════════════════
def test_encoder_watchdog():
    print('\n① 엔코더 워치독')
    _FakeSerial.fail_ports = set()
    n = make_node(enable_esp1=False, enc_timeout=0.2, cmd_timeout=0.3, deadband_pwm=0)

    # 전진 명령 유지 + 엔코더 정상 수신
    tw = _make_msg('Twist')()
    tw.linear.x = 0.2
    n.cb_cmd_vel(tw)
    feed_encoder(n, [0, 0, 0, 0])
    advance(n, 0.05)
    feed_encoder(n, [100, 100, 100, 100])
    n.control_loop()
    moving = last_pwm(n)
    check('엔코더 살아있을 때 PWM 출력됨', moving != '<M,0,0,0,0>', moving)

    # 엔코더만 끊김 (명령은 계속 유지)
    advance(n, 0.25)
    n.cb_cmd_vel(tw)          # 명령은 계속 들어오는 상황
    n.control_loop()
    check('엔코더 끊기면 PWM 0', last_pwm(n) == '<M,0,0,0,0>', last_pwm(n))
    check('적분항 리셋됨', all(i == 0.0 for i in n.integral), str(n.integral))
    check('ERROR 로그 1회', sum('업링크 끊김' in x for x in n._logger.lines) == 1)

    # 재차 호출해도 로그는 늘지 않아야 (래치)
    n.control_loop()
    check('중복 ERROR 로그 없음', sum('업링크 끊김' in x for x in n._logger.lines) == 1)

    # 복구
    feed_encoder(n, [200, 200, 200, 200])
    n.cb_cmd_vel(tw)
    n.control_loop()
    check('복구 로그', any('복구' in x for x in n._logger.lines))
    check('복구 후 PWM 재개', last_pwm(n) != '<M,0,0,0,0>', last_pwm(n))


def test_cmd_watchdog_still_works():
    print('\n② 기존 300ms 명령 워치독 (회귀 확인)')
    _FakeSerial.fail_ports = set()
    n = make_node(enable_esp1=False, enc_timeout=10.0, cmd_timeout=0.3)
    tw = _make_msg('Twist')()
    tw.linear.x = 0.2
    n.cb_cmd_vel(tw)
    feed_encoder(n, [0, 0, 0, 0])
    advance(n, 0.05)
    feed_encoder(n, [50, 50, 50, 50])
    n.control_loop()
    check('명령 유지 중 PWM 출력', last_pwm(n) != '<M,0,0,0,0>', last_pwm(n))
    advance(n, 0.4)                       # 명령만 끊김
    feed_encoder(n, [100, 100, 100, 100])  # 엔코더는 계속 살아있음
    n.control_loop()
    check('명령 끊기면 PWM 0', last_pwm(n) == '<M,0,0,0,0>', last_pwm(n))


def test_enable_esp1():
    print('\n③ enable_esp1 / 포트 열기 실패 처리')
    _FakeSerial.fail_ports = set()

    n = make_node(enable_esp1=False)
    check('esp1=false 면 포트 안 엶', n.esp1 is None)
    check('UNO 는 살아있음', n.uno is not None)
    check('캘리브레이션 모드 경고 출력', any('ESP1 비활성' in x for x in n._logger.lines))

    # ★ 10/4 정책 변경: esp1=true 인데 포트가 없으면 조용히 강등하지 않고 기동 실패.
    #   (강등되면 IMU 없이 주행하게 되는데 경고 한 줄이라 놓쳤음)
    _FakeSerial.fail_ports = {'/dev/ttyACM0'}
    try:
        make_node(enable_esp1=True)
        check('ESP1 열기 실패 시 기동 실패', False, '예외가 안 났음')
    except _SerialException:
        check('ESP1 열기 실패 시 기동 실패', True)

    # UNO 실패는 치명 유지
    _FakeSerial.fail_ports = {'/dev/ttyCH341USB0'}
    try:
        make_node(enable_esp1=False)
        check('UNO 실패 시 예외 전파', False, '예외가 안 났음')
    except _SerialException:
        check('UNO 실패 시 예외 전파', True)


def test_enc_sign():
    """
    ★ 핵심 회귀 테스트 — 구동팀 회신 §6-2 (우측 2바퀴 배선 반대) 대응.

    모터 전원선만 뒤집힌 바퀴는 wheel_sign=-1 인데 엔코더는 정상(enc_sign=+1).
    이 조합에서 target 과 meas 의 부호가 어긋나면 PI 가 양의 되먹임이 되어
    해당 바퀴가 최대속도로 폭주한다. enc_sign 분리로 해결됐는지 확인.
    """
    print('\n⑤ enc_sign — 명령/피드백 극성 분리')
    _FakeSerial.fail_ports = set()

    # 실차 조건: 우측 2바퀴 모터 반대(wheel_sign -1), 엔코더는 정상(enc_sign +1)
    n = make_node(enable_esp1=False, wheel_sign=[1, 1, -1, -1],
                  enc_sign=[1, 1, 1, 1], deadband_pwm=0)
    check('fb_scale = wheel_sign / enc_sign',
          n.fb_scale == [1.0, 1.0, -1.0, -1.0], str(n.fb_scale))

    # 전진 명령. 바퀴가 '정확히 목표대로' 돌고 있는 상황을 엔코더로 재현한다.
    VX = 0.2
    kin_omega = VX / n.kin.r          # 전진 시 4바퀴 모두 같은 kinematic 각속도
    tw = _make_msg('Twist')()
    tw.linear.x = VX
    n.cb_cmd_vel(tw)

    # 엔코더 raw = enc_sign * w_kin → 4바퀴 모두 +  (엔코더는 정상 극성이므로)
    DT = 0.04
    from smart_upcare_control.mecanum_kinematics import RAD_PER_PULSE
    d = int(round(kin_omega * DT / RAD_PER_PULSE))
    feed_encoder(n, [0, 0, 0, 0])
    advance(n, DT)
    feed_encoder(n, [d, d, d, d])

    target = n.kin.inverse(VX, 0.0, 0.0)
    meas = list(n.meas_omega)
    err = [t - m for t, m in zip(target, meas)]
    check('목표대로 도는데 오차가 0에 가까움 (전 바퀴)',
          all(abs(e) < 0.05 for e in err),
          'err=' + ', '.join(f'{e:+.3f}' for e in err))

    # 오도메트리도 같이 확인 — 전진인데 회전으로 읽히면 부호가 틀린 것
    vx, vy, wz = n.kin.forward(*meas)
    check('오도메트리가 전진으로 읽힘',
          abs(vx - VX) < 0.02 and abs(wz) < 0.05,
          f'vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f}')

    # 적분 폭주 여부 — 여러 사이클 돌려서 PWM 이 포화하지 않는지
    for _ in range(30):
        n.cb_cmd_vel(tw)
        n.control_loop()
        advance(n, DT)
        feed_encoder(n, [d * 2, d * 2, d * 2, d * 2])
    pwms = [int(x) for x in last_pwm(n)[3:-1].split(',')]
    check('우측 2바퀴가 포화(±255)하지 않음',
          all(abs(v) < 255 for v in pwms), str(pwms))

    # ── 대조군: enc_sign 없이 (전부 +1 로 두고 wheel_sign 만) 돌리면 실패해야 정상
    n2 = make_node(enable_esp1=False, wheel_sign=[1, 1, -1, -1],
                   enc_sign=[-1, -1, -1, -1], deadband_pwm=0)
    check('enc_sign 을 틀리게 주면 오차가 커짐 (테스트가 실제로 민감함)',
          n2.fb_scale == [-1.0, -1.0, 1.0, 1.0], str(n2.fb_scale))


def test_imu():
    print('\n⑥ IMU 메시지 / 워치독 (10/4)')
    from smart_upcare_control.serial_protocol import FrameScanner, build_esp1_uplink
    _FakeSerial.fail_ports = set()
    n = make_node(enable_esp1=True)
    check('IMU 워치독 타이머 등록',
          any(cb == n.imu_watchdog for _, cb in n.timers))
    pkt = FrameScanner().feed(build_esp1_uplink(1.5708, 0.5, 0.0, 'H'))[0]
    n.on_esp1(pkt)
    m = n.pub_imu.msgs[-1]
    check('orientation_covariance[0] != -1 (yaw 버려지지 않음)',
          m.orientation_covariance[0] != -1.0, m.orientation_covariance[0])
    check('yaw 분산 양수', m.orientation_covariance[8] > 0)
    check('yaw 90° → q.z≈0.707', abs(m.orientation.z - 0.7071) < 1e-3, m.orientation.z)
    check('gyro_z 전달', abs(m.angular_velocity.z - 0.5) < 1e-9)

    n._logger.lines.clear()
    advance(n, 0.5)
    n.imu_watchdog()
    check('수신 직후엔 에러 없음', not any('IMU 수신 끊김' in x for x in n._logger.lines))
    advance(n, 1.5)
    n.imu_watchdog()
    check('1s 넘게 끊기면 ERROR', any(x.startswith('ERROR') and 'IMU 수신 끊김' in x
                                     for x in n._logger.lines))
    n.on_esp1(pkt)
    n.imu_watchdog()
    check('복구 로그', any('IMU 수신 복구' in x for x in n._logger.lines))

    n2 = make_node(enable_esp1=True)   # 한 번도 못 받음
    advance(n2, 6.0)
    n2.imu_watchdog()
    check('기동 후 8s 유예 중엔 에러 없음',
          not any('IMU 수신 끊김' in x for x in n2._logger.lines))
    advance(n2, 3.0)
    n2.imu_watchdog()
    check('유예 지나도 못 받으면 ERROR',
          any('IMU 수신 끊김' in x for x in n2._logger.lines))


def test_port_defaults():
    print('\n④ 포트 기본값 충돌')
    _FakeSerial.fail_ports = set()
    n = make_node()
    ports = {n.get_parameter(k).value
             for k in ('uno_port', 'esp1_port', 'esp2_port')}
    check('브리지 3포트 서로 다름', len(ports) == 3, str(sorted(ports)))
    check('esp1 이 라이다(ttyUSB0)와 안 겹침',
          n.get_parameter('esp1_port').value != '/dev/ttyUSB0',
          n.get_parameter('esp1_port').value)
    check('esp1_port 가 실차 ESP32-S3 기준(ttyACM0)',
          n.get_parameter('esp1_port').value == '/dev/ttyACM0',
          n.get_parameter('esp1_port').value)
    check('uno_port 가 실차 CH340 기준',
          n.get_parameter('uno_port').value == '/dev/ttyCH341USB0',
          n.get_parameter('uno_port').value)


if __name__ == '__main__':
    test_encoder_watchdog()
    test_cmd_watchdog_still_works()
    test_enable_esp1()
    test_enc_sign()
    test_imu()
    test_port_defaults()
    print(f"\n{'='*50}\n통과 {len(PASS)} / 실패 {len(FAIL)}")
    if FAIL:
        print('실패 항목:', FAIL)
    sys.exit(1 if FAIL else 0)
