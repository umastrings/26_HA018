#!/usr/bin/env python3
"""
demo_scenario_node.py — 데모 시퀀스 오케스트레이션

  복도 진입 → 침상 옆 정지 → 리프트 상승 → 180° 제자리 회전 → 전진 퇴출 → 리프트 하강

  - 이동: Nav2 NavigateToPose 액션 (웨이포인트)
  - 회전: /cmd_vel 에 각속도 인가 (Collision Monitor가 게이팅 → cmd_vel_smoothed → 브리지)
          워치독(300ms) 때문에 회전 구간 동안 20Hz로 연속 발행
  - 리프트: /lift_cmd_mm (Float32) 목표높이 발행
      ★ TODO §6: ESP32 리프트 명령 다운링크 포맷 미확정(효선 언니).
        지금은 목표높이 토픽만 쏨. 확정되면 serial_bridge에 ESP32 다운링크 추가하고
        이 토픽을 거기에 연결. 시뮬/가상 검증은 virtual_lift로 대체 가능.

  실행: ros2 launch smart_upcare_control demo_scenario.launch.py
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from nav2_msgs.action import NavigateToPose


class DemoScenario(Node):
    def __init__(self):
        super().__init__('demo_scenario_node')

        # ── 웨이포인트 (map 좌표) — TODO: 실제 복도/침상 좌표로 교체 ──
        # care_facility.world 기준 근사치 (x 전진 방향)
        self.declare_parameter('entry_xy', [-3.0, 0.0])     # 복도 진입점
        self.declare_parameter('bedside_xy', [1.5, -0.3])   # 침상 옆
        self.declare_parameter('exit_xy', [-3.5, 0.0])      # 퇴출점
        self.declare_parameter('lift_up_mm', 700.0)
        self.declare_parameter('lift_down_mm', 100.0)
        self.declare_parameter('rotate_speed', 0.5)         # rad/s
        self.declare_parameter('lift_wait_s', 4.0)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_lift = self.create_publisher(Float32, 'lift_cmd_mm', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    # ------------------------------------------------------------------
    def run(self):
        p = self.get_parameter
        self.get_logger().info('데모 시퀀스 시작')
        self.nav.wait_for_server()

        entry = p('entry_xy').value
        bedside = p('bedside_xy').value
        exit_ = p('exit_xy').value

        self.get_logger().info('① 복도 진입')
        self.goto(entry[0], entry[1], 0.0)

        self.get_logger().info('② 침상 옆 정지')
        self.goto(bedside[0], bedside[1], 0.0)

        self.get_logger().info('③ 리프트 상승')
        self.set_lift(float(p('lift_up_mm').value))
        time.sleep(float(p('lift_wait_s').value))

        self.get_logger().info('④ 180° 제자리 회전')
        self.rotate(math.pi, float(p('rotate_speed').value))

        self.get_logger().info('⑤ 전진 퇴출')
        self.goto(exit_[0], exit_[1], math.pi)

        self.get_logger().info('⑥ 리프트 하강')
        self.set_lift(float(p('lift_down_mm').value))
        time.sleep(float(p('lift_wait_s').value))

        self.get_logger().info('데모 시퀀스 완료')

    # ------------------------------------------------------------------
    def goto(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = ps
        fut = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('  목표 거부됨')
            return
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)

    def rotate(self, angle, speed):
        """제자리 회전 — 워치독 만족 위해 20Hz 연속 발행"""
        duration = abs(angle) / speed
        tw = Twist()
        tw.angular.z = math.copysign(speed, angle)
        rate_dt = 0.05
        n = int(duration / rate_dt)
        for _ in range(n):
            self.pub_cmd.publish(tw)
            time.sleep(rate_dt)
        self.pub_cmd.publish(Twist())   # 정지

    def set_lift(self, mm):
        msg = Float32()
        msg.data = float(mm)
        self.pub_lift.publish(msg)


def main():
    rclpy.init()
    node = DemoScenario()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
