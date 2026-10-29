#!/usr/bin/env python3
"""care_manager: 여러 입력을 모아 로봇 상태 하나(/care/state)를 정한다.

우선순위: ESTOP > EMERGENCY > BEDSORE > PATROL > IDLE

입력
  /care/estop_hw (Bool)  : 비상정지 버튼 (LED 나노 → care_led_bridge)
  /care/estop_sw (Bool)  : 게임패드 정지 (True=정지, False=해제)
  /care/bedsore  (Bool)  : 욕창 위험 알림 (care_bedsore_watch → 대시보드 판정 그대로)
                           래치하지 않음: 대시보드에서 확인(ack)하거나 자세가 바뀌면 자동으로 풀림
  /care/fall     (Bool)  : 낙상 감지 (현재 시연에서는 사용 안 함, 나중을 위해 남겨 둠)
  /care/mode     (String): 평소 모드 IDLE / PATROL
  /care/reset    (Bool)  : True → 낙상 비상 해제

출력
  /care/state (String) : IDLE / PATROL / BEDSORE / EMERGENCY / ESTOP  (바뀌면 즉시 + 0.5초마다)
  /care/estop (Bool)   : 버튼·게임패드 중 하나라도 정지면 True (시리얼 브리지가 모터 0 고정에 사용)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class CareManager(Node):
    def __init__(self):
        super().__init__('care_manager')
        self.estop_hw = False
        self.estop_sw = False
        self.fall_latched = False
        self.bedsore = False
        self.mode = 'IDLE'
        self.state = None

        self.pub_state = self.create_publisher(String, '/care/state', 10)
        self.pub_estop = self.create_publisher(Bool, '/care/estop', 10)

        self.create_subscription(Bool, '/care/estop_hw', self.on_estop_hw, 10)
        self.create_subscription(Bool, '/care/estop_sw', self.on_estop_sw, 10)
        self.create_subscription(Bool, '/care/bedsore', self.on_bedsore, 10)
        self.create_subscription(Bool, '/care/fall', self.on_fall, 10)
        self.create_subscription(String, '/care/mode', self.on_mode, 10)
        self.create_subscription(Bool, '/care/reset', self.on_reset, 10)

        self.create_timer(0.5, self.publish)
        self.update()
        self.get_logger().info('care_manager 시작 (초기 상태 IDLE)')

    # ---------- 입력 ----------
    def on_estop_hw(self, msg):
        self.estop_hw = msg.data
        self.update()

    def on_estop_sw(self, msg):
        self.estop_sw = msg.data
        self.update()

    def on_bedsore(self, msg):
        if msg.data != self.bedsore:
            self.get_logger().info('욕창 위험 알림 ' + ('발생' if msg.data else '해제'))
        self.bedsore = msg.data
        self.update()

    def on_fall(self, msg):
        if msg.data and not self.fall_latched:
            self.get_logger().warn('낙상 감지 → EMERGENCY')
            self.fall_latched = True
            self.update()

    def on_mode(self, msg):
        m = msg.data.strip().upper()
        if m in ('IDLE', 'PATROL'):
            self.mode = m
            self.update()
        else:
            self.get_logger().warn(f'모드는 IDLE/PATROL만 가능: {msg.data}')

    def on_reset(self, msg):
        if msg.data and self.fall_latched:
            self.get_logger().info('낙상 비상 해제')
            self.fall_latched = False
            self.update()

    # ---------- 상태 결정 ----------
    def decide(self):
        if self.estop_hw or self.estop_sw:
            return 'ESTOP'
        if self.fall_latched:
            return 'EMERGENCY'
        if self.bedsore:
            return 'BEDSORE'
        return self.mode

    def update(self):
        new = self.decide()
        if new != self.state:
            self.get_logger().info(f'상태 {self.state} → {new}')
            self.state = new
        self.publish()

    def publish(self):
        self.pub_state.publish(String(data=self.state))
        self.pub_estop.publish(Bool(data=self.estop_hw or self.estop_sw))


def main():
    rclpy.init()
    node = CareManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
