"""
메카넘 휠 역기구학 / 정기구학 — 순수 계산 모듈 (ROS 비의존)

좌표계: ROS REP-103
  x: 전방(+), y: 좌측(+), z: 상방(+), wz: 반시계(+)

바퀴 배치 (위에서 본 그림):
        앞(+x)
    M1        M4
      \      /        <- 롤러 방향 (X자, 표준 배치 가정)
      /      \
    M2        M3
        뒤(-x)

  M1 = 좌상(front-left)   M4 = 우상(front-right)
  M2 = 좌하(rear-left)    M3 = 우하(rear-right)

  ※ 이 배치/롤러 방향은 02번 문서 기준. 실물 검증 필수.
     다르면 config의 WHEEL_SIGN 으로 흡수.
"""


class MecanumKinematics:
    def __init__(
        self,
        wheel_radius: float = 0.040145,  # 지름 80.29mm STEP 실측 (7/29). ★CAD 외경 — 하중 시 눌림 → 조립 후 직선 2m 보정
        track_width: float = 0.1354,     # 좌우 바퀴 축 중심 간 거리. 90도 회전 odom 재보정값 (CLAUDE.md 참조)
        wheelbase: float = 0.19205,      # 앞뒤 바퀴 축 중심 간 거리 (STEP 실측 7/29)
        wheel_sign=(1, 1, 1, 1),      # M1~M4 회전 부호 플립 (실물 검증 후 확정)
    ):
        self.r = wheel_radius
        # K = (L + W) : L=휠베이스/2, W=트랙폭/2
        self.L = wheelbase / 2.0
        self.W = track_width / 2.0
        self.K = self.L + self.W
        self.sign = list(wheel_sign)

    # ------------------------------------------------------------------
    # 역기구학: (vx, vy, wz) -> 4바퀴 각속도 [rad/s]
    # ------------------------------------------------------------------
    def inverse(self, vx: float, vy: float, wz: float):
        """
        표준 메카넘 역기구학 (X-configuration 롤러 기준)

            w1 = (vx - vy - K*wz) / r     M1 좌상
            w2 = (vx + vy - K*wz) / r     M2 좌하
            w3 = (vx - vy + K*wz) / r     M3 우하
            w4 = (vx + vy + K*wz) / r     M4 우상

        검증 포인트:
          - 순수 전진 (vx>0)       -> 4바퀴 모두 같은 부호 (+,+,+,+)
          - 순수 좌이동 (vy>0)     -> X자 패턴 (-,+,-,+)
          - 순수 좌회전 (wz>0)     -> 좌측 후진, 우측 전진 (-,-,+,+)
        """
        w1 = (vx - vy - self.K * wz) / self.r
        w2 = (vx + vy - self.K * wz) / self.r
        w3 = (vx - vy + self.K * wz) / self.r
        w4 = (vx + vy + self.K * wz) / self.r

        w = [w1, w2, w3, w4]
        return [s * wi for s, wi in zip(self.sign, w)]

    # ------------------------------------------------------------------
    # 정기구학: 4바퀴 각속도 [rad/s] -> (vx, vy, wz)
    # ------------------------------------------------------------------
    def forward(self, w1: float, w2: float, w3: float, w4: float):
        """
        역기구학의 의사역행렬 (4x3 -> 3x4)

            vx = (w1 + w2 + w3 + w4) * r / 4
            vy = (-w1 + w2 - w3 + w4) * r / 4
            wz = (-w1 - w2 + w3 + w4) * r / (4*K)
        """
        # 부호 플립 되돌리기 (센서 -> 기구학 좌표계)
        w1, w2, w3, w4 = [wi / s for s, wi in zip(self.sign, (w1, w2, w3, w4))]

        vx = (w1 + w2 + w3 + w4) * self.r / 4.0
        vy = (-w1 + w2 - w3 + w4) * self.r / 4.0
        wz = (-w1 - w2 + w3 + w4) * self.r / (4.0 * self.K)
        return vx, vy, wz


# ----------------------------------------------------------------------
# 엔코더 상수 (감속비 90:1 확정)
# ----------------------------------------------------------------------
PPR = 12                       # 엔코더 base pulse per rev
QUAD = 4                       # quadrature (AB 2상)
GEAR_RATIO = 90                # ★ 확정
CPR = PPR * QUAD * GEAR_RATIO  # 4320 pulse / 바퀴 1회전

WHEEL_DIAMETER = 0.08029     # STEP 실측 7/29 (기존 0.082 가정). CAD 외경 — 조립 후 보정 대상
import math
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER      # 0.25224 m
DIST_PER_PULSE = WHEEL_CIRCUMFERENCE / CPR          # 5.8389e-5 m/pulse (휠 지름 정정분)
RAD_PER_PULSE = 2.0 * math.pi / CPR                 # 1.4544e-3 rad/pulse


def pulses_to_wheel_omega(delta_pulses: int, dt: float) -> float:
    """엔코더 델타 펄스 -> 바퀴 각속도 [rad/s]"""
    if dt <= 0.0:
        return 0.0
    return (delta_pulses * RAD_PER_PULSE) / dt


if __name__ == "__main__":
    print(f"CPR              = {CPR} pulse/rev")
    print(f"wheel circumf.   = {WHEEL_CIRCUMFERENCE:.5f} m")
    print(f"DIST_PER_PULSE   = {DIST_PER_PULSE:.4e} m/pulse")
    print(f"RAD_PER_PULSE    = {RAD_PER_PULSE:.4e} rad/pulse")

    kin = MecanumKinematics()
    print(f"\nK = L + W        = {kin.K:.4f} m")

    print("\n--- 역기구학 검증 ---")
    cases = [
        ("전진   vx=+0.25", (0.25, 0.0, 0.0)),
        ("좌이동 vy=+0.15", (0.0, 0.15, 0.0)),
        ("좌회전 wz=+0.5 ", (0.0, 0.0, 0.5)),
        ("대각선 vx=vy=0.1", (0.1, 0.1, 0.0)),
    ]
    for name, (vx, vy, wz) in cases:
        w = kin.inverse(vx, vy, wz)
        print(f"{name} -> " + "  ".join(f"{x:+7.3f}" for x in w))

    print("\n--- 왕복 검증 (inverse -> forward) ---")
    for name, cmd in cases:
        w = kin.inverse(*cmd)
        back = kin.forward(*w)
        err = max(abs(a - b) for a, b in zip(cmd, back))
        print(f"{name} -> vx={back[0]:+.4f} vy={back[1]:+.4f} wz={back[2]:+.4f}  (err={err:.2e})")

    print("\n--- 정격 속도 한계 ---")
    rated_rpm = 60
    rated_omega = rated_rpm * 2 * math.pi / 60
    print(f"정격 60rpm = {rated_omega:.3f} rad/s = {rated_omega * kin.r:.3f} m/s (바퀴 선속도)")
    w = kin.inverse(0.25, 0.0, 0.0)
    print(f"vx=0.25 명령 시 최대 바퀴 각속도 = {max(abs(x) for x in w):.3f} rad/s "
          f"({'OK' if max(abs(x) for x in w) <= rated_omega else 'OVER'})")
    w = kin.inverse(0.0, 0.15, 0.0)
    print(f"vy=0.15 명령 시 최대 바퀴 각속도 = {max(abs(x) for x in w):.3f} rad/s "
          f"({'OK' if max(abs(x) for x in w) <= rated_omega else 'OVER'})")
