#!/usr/bin/env python3
"""
virtual_lift_publisher.py — 리프트 하드웨어 없이 /lift_mm 모사

  발행: /lift_mm  (std_msgs/Float32)  [mm]

  목적 (§3): 리프트 실물(효선 언니) 기다리지 않고 lift_tf_node + Dynamic TF를
             선행 검증. 시뮬/실물 어느 쪽이든 lift_tf_node 입력을 동일하게 흉내냄.

  모드:
    cycle : min↔max 를 speed[mm/s]로 왕복하는 삼각파 (기본)
    hold  : hold_mm 고정값 유지

  Dynamic TF 지연/오차 측정(§3)용으로, 발행 로그에 목표 높이를 남길 수 있음.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class VirtualLiftPublisher(Node):
    def __init__(self):
        super().__init__('virtual_lift_publisher')

        self.declare_parameter('mode', 'cycle')       # 'cycle' | 'hold'
        self.declare_parameter('min_mm', 100.0)       # §6 리프트 가동범위
        self.declare_parameter('max_mm', 740.0)
        self.declare_parameter('speed_mms', 50.0)     # 왕복 속도 [mm/s]
        self.declare_parameter('hold_mm', 400.0)
        self.declare_parameter('rate', 30.0)

        p = self.get_parameter
        self.mode = p('mode').value
        self.min_mm = float(p('min_mm').value)
        self.max_mm = float(p('max_mm').value)
        self.speed = float(p('speed_mms').value)
        self.hold_mm = float(p('hold_mm').value)
        self.rate = float(p('rate').value)

        self.value = self.min_mm
        self.direction = 1.0

        self.pub = self.create_publisher(Float32, 'lift_mm', 10)
        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f"virtual lift: mode={self.mode} "
            f"[{self.min_mm}~{self.max_mm}]mm @ {self.speed}mm/s")

    def tick(self):
        if self.mode == 'hold':
            self.value = self.hold_mm
        else:  # cycle (삼각파)
            self.value += self.direction * self.speed * self.dt
            if self.value >= self.max_mm:
                self.value = self.max_mm
                self.direction = -1.0
            elif self.value <= self.min_mm:
                self.value = self.min_mm
                self.direction = 1.0

        msg = Float32()
        msg.data = float(self.value)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = VirtualLiftPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
