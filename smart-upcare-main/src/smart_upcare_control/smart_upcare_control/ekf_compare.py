#!/usr/bin/env python3
"""
ekf_compare.py — EKF 효과를 눈으로 확인하는 비교 도구

  /odom_raw          (엔코더만)      ─┐
                                      ├─→ 나란히 비교
  /odometry/filtered (EKF 융합)      ─┘

사용법:
  ros2 run smart_upcare_control ekf_compare
  또는
  python3 ~/ekf_compare.py          (source 후)

  터미널에 실시간 표가 뜬다.

핵심 관전 포인트:
  1) fake_uno 를 --noise 없이 → 둘이 거의 같아야 정상
  2) fake_uno --noise 5 로 재시작 → raw 가 요동치는데
     filtered 는 매끄러워야 한다. 이게 EKF 가 일하는 증거.
  3) 회전 명령 → raw 의 yaw 와 filtered 의 yaw 비교.
     ★ 메카넘은 슬립 때문에 엔코더 yaw 가 틀어지는데,
       filtered 는 IMU 를 따라가야 한다.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


class EkfCompare(Node):
    def __init__(self):
        super().__init__('ekf_compare')
        self.raw = None
        self.flt = None
        self.imu_yaw = None

        self.create_subscription(Odometry, '/odom_raw', self.cb_raw, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.cb_flt, 10)
        self.create_subscription(Imu, '/imu/data', self.cb_imu, 10)
        self.create_timer(0.5, self.report)

        self.n_raw = 0
        self.n_flt = 0
        print("\n" + "=" * 78)
        print("  EKF 비교 — /odom_raw (엔코더)  vs  /odometry/filtered (EKF)")
        print("=" * 78)

    def cb_raw(self, m):
        self.raw = m
        self.n_raw += 1

    def cb_flt(self, m):
        self.flt = m
        self.n_flt += 1

    def cb_imu(self, m):
        self.imu_yaw = yaw_of(m.orientation)

    def report(self):
        if self.raw is None:
            print("  ⏳ /odom_raw 대기중... (fake_uno + serial_bridge 켜져 있나?)")
            return
        if self.flt is None:
            print(f"  ⏳ /odometry/filtered 대기중... (EKF 켜져 있나?)  raw={self.n_raw}")
            return

        r, f = self.raw, self.flt
        rx, ry = r.pose.pose.position.x, r.pose.pose.position.y
        fx, fy = f.pose.pose.position.x, f.pose.pose.position.y
        ryaw = math.degrees(yaw_of(r.pose.pose.orientation))
        fyaw = math.degrees(yaw_of(f.pose.pose.orientation))
        iyaw = math.degrees(self.imu_yaw) if self.imu_yaw is not None else float('nan')

        rvx, rvy = r.twist.twist.linear.x, r.twist.twist.linear.y
        fvx, fvy = f.twist.twist.linear.x, f.twist.twist.linear.y
        rwz, fwz = r.twist.twist.angular.z, f.twist.twist.angular.z

        drift = math.hypot(fx - rx, fy - ry)

        print("\033[2J\033[H", end="")  # 화면 지우기
        print("=" * 78)
        print(f"  EKF 비교        raw={self.n_raw}  filtered={self.n_flt}")
        print("=" * 78)
        print(f"{'':14s} {'raw (엔코더)':>18s} {'filtered (EKF)':>18s} {'IMU':>12s}")
        print("-" * 78)
        print(f"{'위치 x [m]':14s} {rx:18.4f} {fx:18.4f}")
        print(f"{'위치 y [m]':14s} {ry:18.4f} {fy:18.4f}")
        print(f"{'방향 yaw [°]':14s} {ryaw:18.2f} {fyaw:18.2f} {iyaw:12.2f}")
        print("-" * 78)
        print(f"{'속도 vx [m/s]':14s} {rvx:18.4f} {fvx:18.4f}")
        print(f"{'속도 vy [m/s]':14s} {rvy:18.4f} {fvy:18.4f}")
        print(f"{'각속도 wz':14s} {rwz:18.4f} {fwz:18.4f}")
        print("=" * 78)
        print(f"  위치 차이(raw vs filtered): {drift:.4f} m")

        if abs(fyaw - iyaw) < 3.0:
            print("  ✅ filtered yaw 가 IMU 를 따라감 (의도대로)")
        else:
            print(f"  ⚠️  filtered yaw 와 IMU 차이 {abs(fyaw-iyaw):.1f}° — 설정 확인")

        print()
        print("  💡 fake_uno 를 --noise 5 로 재시작해보면:")
        print("     raw 속도가 요동치는데 filtered 는 매끄러워야 함 = EKF 작동 증거")


def main():
    rclpy.init()
    node = EkfCompare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
