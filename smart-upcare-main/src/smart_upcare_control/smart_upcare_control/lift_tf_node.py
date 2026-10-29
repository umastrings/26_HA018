#!/usr/bin/env python3
"""
lift_tf_node.py — lift_mm -> JointState -> prismatic TF (Dynamic TF)

  구독: /lift_mm       (std_msgs/Float32)  리프트 높이 [mm]
  발행: /joint_states  (sensor_msgs/JointState)  prismatic 조인트 위치 [m]

  → robot_state_publisher 가 이 joint_state + URDF로
    base_link -> camera_mount_link -> camera_link 의 동적 TF를 굴린다.
    (이 노드는 TF를 직접 broadcast하지 않는다 — RSP가 소유)

★ 전제 (task 6 URDF 작업)
  URDF의 `camera_mount_joint` 가 type="fixed" → "prismatic" 으로 바뀌어야
  RSP가 이 조인트를 움직인다. 현재 xacro 주석의 "A안 핵심 확장 지점".

★ 미확정 — §6 (효선 언니 기준면 확정 대기)
  lift_mm 이 "지면 기준 절대높이"인지 "접힘 기준 변위"인지 미확정.
    - 절대높이 모드(lift_mm_is_absolute=True):
        joint_pos = lift_mm/1000 - lift_min_height   (URDF 원점이 접힘높이라 상쇄)
    - 변위 모드(False):
        joint_pos = lift_mm/1000
  확정되면 파라미터만 바꾸면 됨.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class LiftTfNode(Node):
    def __init__(self):
        super().__init__('lift_tf_node')

        self.declare_parameter('joint_name', 'camera_mount_joint')
        self.declare_parameter('lift_mm_is_absolute', True)   # TODO §6 확정
        self.declare_parameter('lift_min_height', 0.092)      # URDF lift_min_height [m]
        self.declare_parameter('stroke', 0.648)               # 가동 스트로크 [m]
        self.declare_parameter('publish_rate', 30.0)          # 미수신 시에도 최신값 유지 발행

        p = self.get_parameter
        self.joint_name = p('joint_name').value
        self.is_absolute = bool(p('lift_mm_is_absolute').value)
        self.min_h = float(p('lift_min_height').value)
        self.stroke = float(p('stroke').value)

        self.joint_pos = 0.0     # 현재 prismatic 위치 [m]
        self.have_data = False

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(Float32, 'lift_mm', self.cb_lift, 10)
        self.create_timer(1.0 / float(p('publish_rate').value), self.publish_js)

        self.get_logger().info(
            f"lift_tf: joint='{self.joint_name}' "
            f"mode={'absolute' if self.is_absolute else 'displacement'} "
            f"min_h={self.min_h} stroke={self.stroke}")

    def cb_lift(self, msg: Float32):
        m = msg.data / 1000.0
        pos = (m - self.min_h) if self.is_absolute else m
        # prismatic 한계로 클램프 (센서 튐/기준면 오차 방어)
        self.joint_pos = max(0.0, min(self.stroke, pos))
        self.have_data = True

    def publish_js(self):
        if not self.have_data:
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        js.position = [self.joint_pos]
        self.pub.publish(js)


def main():
    rclpy.init()
    node = LiftTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
