#!/usr/bin/env python3
"""
tf_lag_check.py — 시뮬 시간 대비 /scan, /tf 지연 진단

  "Message Filter dropping message ... the timestamp on the message is earlier
   than all the data in the transform cache" 의 원인을 찾기 위한 도구.

  체크 항목
    1) /clock 대비 /scan 지연
    2) /clock 대비 각 TF(부모->자식) 지연  ← 음수면 그 노드가 미래 시각을 쏘는 것
    3) map -> odom TF 존재 여부           ← 없으면 AMCL 이 스캔을 못 받고 있음
    4) /amcl_pose 수신 여부

  실행 (공유폴더에서 바로):
    python3 /media/sf_hanium/hanium_ws/tools/tf_lag_check.py
"""
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped

DURATION = 12.0


def stamp(v):
    return v.sec + v.nanosec * 1e-9


def main():
    rclpy.init()
    node = rclpy.create_node('tf_lag_check')

    # /clock 과 /scan 은 BEST_EFFORT 로 발행되는 경우가 있어 맞춰준다
    qos_be = QoSProfile(depth=10,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST,
                        durability=DurabilityPolicy.VOLATILE)
    qos_rel = QoSProfile(depth=50,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)

    state = {'clock': None, 'scan': None, 'scan_n': 0, 'amcl_n': 0}
    tfs = {}      # "부모 -> 자식" : (마지막 stamp, 개수)

    def on_clock(m):
        state['clock'] = stamp(m.clock)

    def on_scan(m):
        state['scan'] = stamp(m.header.stamp)
        state['scan_n'] += 1

    def on_tf(m):
        for tr in m.transforms:
            k = f"{tr.header.frame_id} -> {tr.child_frame_id}"
            prev = tfs.get(k, (0.0, 0))
            tfs[k] = (stamp(tr.header.stamp), prev[1] + 1)

    def on_amcl(m):
        state['amcl_n'] += 1

    # /clock 은 BEST_EFFORT / RELIABLE 양쪽 다 걸어둔다
    node.create_subscription(Clock, '/clock', on_clock, qos_be)
    node.create_subscription(Clock, '/clock', on_clock, qos_rel)
    node.create_subscription(LaserScan, '/scan', on_scan, qos_be)
    node.create_subscription(LaserScan, '/scan', on_scan, qos_rel)
    node.create_subscription(TFMessage, '/tf', on_tf, qos_rel)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_amcl, qos_rel)

    print(f"{DURATION:.0f}초 동안 수집 중...\n")
    t0 = time.time()
    while time.time() - t0 < DURATION and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)

    c = state['clock']
    print("=" * 62)
    if c is None:
        print(" !! /clock 을 못 받음 — 시뮬이 안 떠 있거나 물리가 정지 상태")
        print("    ros2 service call /unpause_physics std_srvs/srv/Empty {}")
        rclpy.shutdown()
        return

    print(f" 시뮬 현재시각  {c:10.3f} s")
    print("=" * 62)

    if state['scan'] is None:
        print(" /scan            수신 없음  <-- 심각")
    else:
        lag = c - state['scan']
        v = "OK" if abs(lag) < 0.5 else ("지연 과다" if lag > 0 else "미래 시각")
        print(f" /scan            {state['scan']:10.3f}  지연 {lag:+7.3f} s  "
              f"({state['scan_n']}개)  {v}")

    print("-" * 62)
    if not tfs:
        print(" /tf 수신 없음  <-- TF 발행자가 하나도 없음")
    for k in sorted(tfs):
        ts, n = tfs[k]
        lag = c - ts
        if lag < -0.5:
            v = "!! 미래 시각 (이 노드가 범인)"
        elif lag > 2.0:
            v = "!! 심하게 지연"
        elif lag > 0.5:
            v = "지연"
        else:
            v = "OK"
        print(f" TF {k:32s} 지연 {lag:+7.3f} s  ({n}개)  {v}")

    print("-" * 62)
    has_map = any(k.startswith('map ->') for k in tfs)
    print(f" map -> odom TF   {'있음' if has_map else '없음  <-- AMCL 이 스캔을 처리하지 못하는 중'}")
    print(f" /amcl_pose       {state['amcl_n']}개 수신"
          f"{'' if state['amcl_n'] else '  <-- AMCL 이 위치추정을 못 하고 있음'}")
    print("=" * 62)

    print("\n[해석]")
    print("  - 어떤 TF 지연이 음수(미래)  -> 그 노드의 use_sim_time 이 false")
    print("  - /scan 지연만 큼            -> Gazebo 센서 발행이 밀림")
    print("  - map -> odom 없음           -> AMCL 이 스캔 못 받음 (원인은 위 두 줄)")
    print("  - 전부 OK 인데 로봇이 안 감  -> 컨트롤러(MPPI) 쪽 문제")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
