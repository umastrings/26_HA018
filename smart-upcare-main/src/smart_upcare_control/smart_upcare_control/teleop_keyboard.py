#!/usr/bin/env python3
"""
teleop_keyboard.py — 메카넘 WASD 키보드 조종 (젯슨에서 실행)

  발행: /cmd_vel (geometry_msgs/Twist)  ← mecanum_ik_node 가 구독

  W/S : 전/후진        (vx)
  A/D : 좌/우 횡이동   (vy, 메카넘 게걸음)
  Q/E : 좌/우 회전     (wz)
  Space 또는 X : 즉시 정지
  + / - : 속도 배율 증감
  Ctrl-C : 종료

  ★ 키를 누르는 동안만 이동, 떼면 자동 정지 (키 리피트 기반, hold_timeout).
    한 번에 한 축만 입력됨(터미널 한계). 대각선은 명령 조합 불가.
"""
import sys
import termios
import tty
import select
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
┌─────────────────────────────────────────────┐
│  Mecanum WASD Teleop                        │
│    W / S : 전진 / 후진                        │
│    A / D : 좌 / 우 횡이동 (게걸음)             │
│    Q / E : 좌 / 우 회전                        │
│    Space / X : 정지                           │
│    + / -  : 속도 배율                          │
│    Ctrl-C : 종료                              │
│  (키를 누르고 있는 동안 이동, 떼면 정지)        │
└─────────────────────────────────────────────┘
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.declare_parameter('vx', 0.12)   # m/s
        self.declare_parameter('vy', 0.10)   # m/s (횡이동은 슬립 커서 보수적)
        self.declare_parameter('wz', 0.6)    # rad/s
        self.declare_parameter('hold_timeout', 0.4)  # s, 키 떼면 정지까지
        self.vx = self.get_parameter('vx').value
        self.vy = self.get_parameter('vy').value
        self.wz = self.get_parameter('wz').value
        self.hold_timeout = self.get_parameter('hold_timeout').value

        self.speed = 1.0
        self.cur = (0.0, 0.0, 0.0)
        self.last_key = self.get_clock().now()

        self.create_timer(0.05, self.tick)   # 20Hz 발행

    def set_cmd(self, vx, vy, wz):
        self.cur = (vx, vy, wz)
        self.last_key = self.get_clock().now()

    def tick(self):
        age = (self.get_clock().now() - self.last_key).nanoseconds * 1e-9
        vx, vy, wz = self.cur if age <= self.hold_timeout else (0.0, 0.0, 0.0)
        m = Twist()
        m.linear.x = float(vx)
        m.linear.y = float(vy)
        m.angular.z = float(wz)
        self.pub.publish(m)


def main():
    rclpy.init()
    node = TeleopKeyboard()
    print(HELP)
    print(f"[속도배율 x{node.speed:.1f}]  vx={node.vx} vy={node.vy} wz={node.wz}")

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            k = sys.stdin.read(1).lower()
            s = node.speed
            if k == 'w':
                node.set_cmd(node.vx * s, 0.0, 0.0)
            elif k == 's':
                node.set_cmd(-node.vx * s, 0.0, 0.0)
            elif k == 'a':
                node.set_cmd(0.0, node.vy * s, 0.0)
            elif k == 'd':
                node.set_cmd(0.0, -node.vy * s, 0.0)
            elif k == 'q':
                node.set_cmd(0.0, 0.0, node.wz * s)
            elif k == 'e':
                node.set_cmd(0.0, 0.0, -node.wz * s)
            elif k in (' ', 'x'):
                node.set_cmd(0.0, 0.0, 0.0)
            elif k in ('+', '='):
                node.speed = min(2.0, node.speed + 0.1)
                sys.stdout.write(f"\r[속도배율 x{node.speed:.1f}]        ")
            elif k == '-':
                node.speed = max(0.1, node.speed - 0.1)
                sys.stdout.write(f"\r[속도배율 x{node.speed:.1f}]        ")
            elif k == '\x03':   # Ctrl-C
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        stop = Twist()
        for _ in range(5):
            node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
