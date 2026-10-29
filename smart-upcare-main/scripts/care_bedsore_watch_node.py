#!/usr/bin/env python3
"""care_bedsore_watch: 대시보드(app.py)의 욕창 판정을 그대로 가져와 ROS로 알린다.

app.py 가 이미 /api/patients 에서 환자별 alert(욕창 위험)를 계산하므로,
같은 판정을 1초마다 읽어 /care/bedsore (std_msgs/Bool) 로 발행한다.
→ 대시보드 화면과 로봇(LED·음성)이 항상 같은 기준으로 움직이고,
  대시보드에서 '확인(ack)'을 누르면 로봇 알림도 같이 풀린다.

주의: patients.db 가 없으면 app.py 가 가짜 데이터(데모 모드)를 보여 주는데,
      가짜 환자는 몇 초 만에 alert 가 켜지므로 데모 모드일 때는 무시한다.

실행: python3 care_bedsore_watch_node.py --ros-args -p url:=http://127.0.0.1:5001/api/patients
"""
import json
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class BedsoreWatch(Node):
    def __init__(self):
        super().__init__('care_bedsore_watch')
        self.declare_parameter('url', 'http://127.0.0.1:5001/api/patients')
        self.declare_parameter('period', 1.0)
        self.declare_parameter('ignore_demo', True)

        self.pub = self.create_publisher(Bool, '/care/bedsore', 10)
        self.pub_info = self.create_publisher(String, '/care/bedsore_info', 10)
        self.last = None
        # 젯슨에 프록시 환경변수가 있어도 localhost 대시보드에 바로 붙도록 프록시를 끈다
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.create_timer(float(self.get_parameter('period').value), self.poll)
        self.get_logger().info(f"대시보드 확인 시작: {self.get_parameter('url').value}")

    def poll(self):
        url = self.get_parameter('url').value
        try:
            with self.opener.open(url, timeout=0.8) as r:
                data = json.loads(r.read().decode())
        except Exception as e:
            # 대시보드가 꺼져 있으면 알림을 끈 상태로 둔다 (울린 채로 멈추지 않게)
            self.get_logger().warn(f'대시보드 응답 없음: {e}', throttle_duration_sec=10.0)
            self.publish(False, '')
            return

        if data.get('demo') and self.get_parameter('ignore_demo').value:
            self.get_logger().warn('대시보드가 데모 모드(가짜 데이터) → 무시',
                                   throttle_duration_sec=30.0)
            self.publish(False, '')
            return

        alerted = [p.get('name', '?') for p in data.get('patients', []) if p.get('alert')]
        self.publish(bool(alerted), ', '.join(alerted))

    def publish(self, alert, names):
        if alert != self.last:
            if alert:
                self.get_logger().warn(f'욕창 위험: {names}')
            elif self.last is not None:
                self.get_logger().info('욕창 위험 해제')
            self.last = alert
        self.pub.publish(Bool(data=alert))
        self.pub_info.publish(String(data=names))


def main():
    rclpy.init()
    node = BedsoreWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
