"""
serial_protocol.py — 3-MCU <-> 젯슨 시리얼 패킷 프로토콜 (순수 모듈, ROS 비의존)

★ v2 (3-MCU 재편)
  기존 v1은 UNO 1대가 엔코더·IMU·리프트를 한 스트림에 실어보내는 전제였음.
  확정 아키텍처는 3-MCU라 스트림이 포트별로 분리됨:

    UNO      : 엔코더 4채널 + 모터 PWM
    ESP32 #1 : BNO055 IMU + 시저 리프트 (lift_mm)
    ESP32 #2 : 섀시폭 스텝모터 (비블로킹)

  → 마커('E'/'I'/'W'/'M')로 스트림을 구분한다. 포트별로 한 종류만 오지만,
    마커를 두면 배선이 바뀌어도 오파싱을 막고 디버깅 로그가 명확해짐.

────────────────────────────────────────────────────────────
[업링크]  MCU -> 젯슨

  UNO      <E,c1,c2,c3,c4>\n
             c1~4 : long  누적 엔코더 카운트 (부호 포함)
                    ★ 델타 아님. 누적 → 패킷 유실돼도 다음 패킷에서 복구
             휠 순서: c1=M1(FL) c2=M2(RL) c3=M3(RR) c4=M4(FR)
             예: <E,12034,-5671,8890,-2234>

  ESP32 #1 <I,yaw,gyro_z,lift_mm,state>\n
             yaw     : float  BNO055 heading [rad]  (CCW +)
             gyro_z  : float  각속도 z [rad/s]
             lift_mm : float  리프트 현재 높이 [mm]
             state   : char   'H'/'M'/'T'(정확히 그 위치) 또는 '-'(이동 중/임의 위치)
                       ★ 통신 안정성 보강: <D,state,checksum>과 같은 정보를 10Hz로
                         중복 전송 (자가 복구용). <D,...> 한 번이 유실돼도 최대
                         0.1초 뒤 이 필드로 /lift/state가 다시 맞춰짐.
             예: <I,0.7854,0.1200,120.5,->

  ESP32 #2 <W,width_mm>\n         (옵션 · 비블로킹)
             width_mm : float  현재 섀시폭 [mm]
             예: <W,394.0>

  ESP32 #1 <D,state,checksum>\n   (리프트 완료 이벤트)
             state    : char  'H' / 'M' / 'T' — 목표 도달 시 1회성으로 전송
             checksum : int   state의 ASCII 코드값 (ord(state))
                        ★ 통신 안정성 보강: 노이즈로 state가 다른 유효한 문자
                          (예: 'H'->'M')로 깨져도, checksum까지 우연히 같이
                          맞아떨어질 확률은 매우 낮아 여기서 걸러짐.
             예: <D,H,72>
             → /lift/state (std_msgs/String, "HOME"/"MID"/"TOP")로 매핑되어
               chassis_controller.py 등 다른 노드가 구독함.

────────────────────────────────────────────────────────────
[다운링크]  젯슨 -> UNO   50Hz

  <M,pwm1,pwm2,pwm3,pwm4>\n
    pwm1~4 : int  바퀴별 PWM  [-255, +255]
             ★ 젯슨측 FF+PI 속도 폐루프의 출력. UNO는 개루프(PWM=16×Setspeed)라
               속도 제어는 전부 젯슨이 담당 → UNO엔 최종 PWM만 내려보냄
             휠 순서: pwm1=M1(FL) pwm2=M2(RL) pwm3=M3(RR) pwm4=M4(FR)
             예: <M,180,-180,180,-180>

────────────────────────────────────────────────────────────
[다운링크]  젯슨 -> ESP32 #1 (리프트)  요청 시에만

  P<mm>\n
    mm : float  목표 리프트 높이 [mm]  (ESP32가 mmToStep()으로 STEP 변환)
         ★ 괄호 프레임(<...>) 아님 — ESP32의 processCommand()가 H/M/T와
           동일하게 평문 한 줄로 파싱하기 때문에 다운링크만 이 형식을 따름.
         예: P450.0

────────────────────────────────────────────────────────────
설계 근거
  - '<' '>' 프레이밍: 스트림 중간부터 붙어도 동기 복구 가능
  - 첫 필드 = 타입 마커: 포트 혼선/오파싱 방지, 로그 가독성
  - CSV 텍스트: MCU에서 sprintf 로 간단, 디버깅 시 눈으로 읽힘
  - 누적 카운트: UDP 성격의 시리얼에서 유실 내성
  - CRC 없음: 짧은 유선 연결 + 마커·필드수 검증으로 충분.
              필요해지면 마지막 필드에 XOR checksum 추가.
"""

START = b'<'
END = b'>'

# 업링크 마커별 (필드 개수) — 마커 필드 제외한 데이터 필드 수
UPLINK_SPEC = {
    'E': 4,   # c1..c4
    'I': 4,   # yaw, gyro_z, lift_mm, state(자가복구용, 통신 안정성 보강)
    'W': 1,   # width_mm
    'D': 2,   # 리프트 완료 이벤트: state, checksum (통신 안정성 보강)
}

MAX_PWM = 255


class PacketStats:
    """파싱 품질 추적 — 실물에서 통신 건전성 판단용"""

    def __init__(self):
        self.ok = 0
        self.bad_format = 0      # 마커/필드 개수/타입 이상
        self.overflow = 0        # 버퍼 넘침 (START 없이 쓰레기 유입)
        self.tick_jump = 0       # 엔코더 카운트 비정상 점프

    def __str__(self):
        total = self.ok + self.bad_format
        rate = (100.0 * self.ok / total) if total else 0.0
        return (f"ok={self.ok} bad={self.bad_format} "
                f"overflow={self.overflow} jump={self.tick_jump} "
                f"({rate:.1f}% good)")


class FrameScanner:
    """
    시리얼 바이트 스트림 -> 패킷 dict (마커 기반)

    스트림 지향: read()로 받은 청크를 계속 feed()하면
    완성된 패킷들을 리스트로 돌려준다. 경계에 걸친 패킷도 처리.

    포트마다 인스턴스 하나씩. 반환 dict의 'type' 키로 어떤 MCU인지 구분.
      E -> {'type':'E', 'ticks':[c1,c2,c3,c4]}
      I -> {'type':'I', 'yaw':.., 'gyro_z':.., 'lift_mm':.., 'coarse_state':..}
      W -> {'type':'W', 'width_mm':..}
      D -> {'type':'D', 'state':..}
    """

    MAX_BUF = 256   # 이보다 길면 프레이밍 깨진 것 → 버림

    def __init__(self, stats: PacketStats = None):
        self.buf = bytearray()
        self.stats = stats or PacketStats()

    def feed(self, chunk: bytes):
        """바이트 청크 투입 -> 완성된 패킷 dict 리스트 반환"""
        out = []
        self.buf.extend(chunk)

        # 버퍼 폭주 방지 (START 없는 쓰레기가 계속 들어오는 경우)
        if len(self.buf) > self.MAX_BUF:
            i = self.buf.rfind(START)
            if i == -1:
                self.buf.clear()
            else:
                self.buf = self.buf[i:]
            self.stats.overflow += 1

        while True:
            s = self.buf.find(START)
            if s == -1:
                # START 없음 → 전부 쓰레기
                self.buf.clear()
                break

            e = self.buf.find(END, s + 1)
            if e == -1:
                # 아직 패킷 미완성 → START 앞의 쓰레기만 버리고 대기
                del self.buf[:s]
                break

            body = bytes(self.buf[s + 1:e])
            del self.buf[:e + 1]

            pkt = self._parse(body)
            if pkt is not None:
                out.append(pkt)

        return out

    def _parse(self, body: bytes):
        try:
            parts = body.decode('ascii', errors='strict').split(',')
        except UnicodeDecodeError:
            self.stats.bad_format += 1
            return None

        if len(parts) < 1:
            self.stats.bad_format += 1
            return None

        marker = parts[0]
        fields = parts[1:]
        spec = UPLINK_SPEC.get(marker)
        if spec is None or len(fields) != spec:
            self.stats.bad_format += 1
            return None

        try:
            pkt = self._decode(marker, fields)
        except (ValueError, IndexError):
            self.stats.bad_format += 1
            return None

        self.stats.ok += 1
        return pkt

    @staticmethod
    def _decode(marker, fields):
        if marker == 'E':
            return {
                'type': 'E',
                'ticks': [int(fields[0]), int(fields[1]),
                          int(fields[2]), int(fields[3])],
            }
        if marker == 'I':
            coarse_state = fields[3]
            if coarse_state not in ('H', 'M', 'T', '-'):
                raise ValueError(coarse_state)
            return {
                'type': 'I',
                'yaw': float(fields[0]),
                'gyro_z': float(fields[1]),
                'lift_mm': float(fields[2]),
                'coarse_state': coarse_state,
            }
        if marker == 'W':
            return {'type': 'W', 'width_mm': float(fields[0])}
        if marker == 'D':
            state = fields[0]
            checksum = int(fields[1])
            # ★ 통신 안정성 보강: checksum이 state의 ASCII 코드와 일치하는지 확인.
            #   'H'가 노이즈로 'M'/'T' 등 다른 유효한 값으로 깨져도, checksum까지
            #   같이 깨질 확률은 매우 낮으므로 이 단계에서 걸러진다.
            if state not in ('H', 'M', 'T') or checksum != ord(state):
                raise ValueError(f"{state},{checksum}")
            return {'type': 'D', 'state': state}
        raise ValueError(marker)


# ----------------------------------------------------------------------
# 다운링크 (젯슨 -> UNO)
# ----------------------------------------------------------------------
def build_motor_downlink(pwm1: int, pwm2: int, pwm3: int, pwm4: int) -> bytes:
    """4바퀴 PWM -> UNO 송신 바이트  <M,pwm1..4>"""
    def clamp(v):
        return max(-MAX_PWM, min(MAX_PWM, int(round(v))))
    return (f"<M,{clamp(pwm1)},{clamp(pwm2)},"
            f"{clamp(pwm3)},{clamp(pwm4)}>\n").encode('ascii')


# ----------------------------------------------------------------------
# 테스트/시뮬용 업링크 빌더 (MCU 흉내)
# ----------------------------------------------------------------------
def build_uno_uplink(ticks) -> bytes:
    return f"<E,{ticks[0]},{ticks[1]},{ticks[2]},{ticks[3]}>\n".encode('ascii')


def build_lift_downlink(mm: float) -> bytes:
    """목표 높이[mm] -> ESP32(리프트) 다운링크 바이트  P<mm>\\n

    ESP32.cpp의 processCommand()가 'P'로 시작하는 문자열을 임의 높이
    이동 명령으로 파싱한다 (mmToStep()으로 STEP 변환 후 이동).
    """
    return f"P{mm:.1f}\n".encode('ascii')


def build_esp1_uplink(yaw: float, gyro_z: float, lift_mm: float, state: str = '-') -> bytes:
    """테스트/시뮬용. state: 'H'/'M'/'T'/'-' (통신 안정성 보강용 자가복구 필드)"""
    return f"<I,{yaw:.4f},{gyro_z:.4f},{lift_mm:.1f},{state}>\n".encode('ascii')


def build_lift_done_uplink(state: str) -> bytes:
    """테스트/시뮬용. <D,state,checksum> — checksum은 state의 ASCII 코드값."""
    return f"<D,{state},{ord(state)}>\n".encode('ascii')


def build_esp2_uplink(width_mm: float) -> bytes:
    return f"<W,{width_mm:.1f}>\n".encode('ascii')
