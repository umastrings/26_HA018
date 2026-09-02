#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import serial
import time
import threading

class IntegratedChassisController(Node):
    def __init__(self):
        super().__init__('integrated_chassis_controller')
        
        # ----------------------------------------------------
        # 1. 시리얼 포트 설정
        # ----------------------------------------------------
        # (1) MKS DLC32 (섀시 제어 보드)
        self.mks_port = '/dev/ttyCH341USB1'
        try:
            self.mks_ser = serial.Serial(self.mks_port, 115200, timeout=0.1)
            self.get_logger().info(f"✅ MKS DLC32 연결 성공: {self.mks_port}")
            time.sleep(1)
            # MKS 보드 초기화
            self.mks_ser.write(b"$20=0\n") # 소프트 리미트 OFF
            self.mks_ser.write(b"$21=0\n") # 하드웨어 강제 알람 OFF
            self.mks_ser.write(b"G91\n")   # 상대 좌표 모드
        except Exception as e:
            self.get_logger().error(f"❌ MKS DLC32 연결 실패: {e}")

        # (2) ESP32 (리프트 높이 수신)
        self.esp_port = '/dev/ttyUSB0' # 필요시 /dev/ttyACM0 등으로 변경
        try:
            self.esp_ser = serial.Serial(self.esp_port, 115200, timeout=0.1)
            self.get_logger().info(f"✅ ESP32 리프트 보드 연결 성공: {self.esp_port}")
        except Exception as e:
            self.get_logger().error(f"❌ ESP32 연결 실패: {e}")
            self.esp_ser = None

        # ----------------------------------------------------
        # 2. 내부 상태 변수 및 ROS 2 토픽 구독
        # ----------------------------------------------------
        self.min_obstacle_dist = 999.0  # 장애물 최소 거리 (m)
        self.current_speed = 0.0        # 로봇 속도 (m/s)
        self.lift_height = 0.0          # 리프트 높이 (mm)
        self.is_limit_pressed = False   # 섀시 리미트 스위치 감지 여부
        
        # ROS 2 토픽 구독
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # ESP32 수신용 별도 스레드 시작
        if self.esp_ser:
            self.esp_thread = threading.Thread(target=self.read_esp32_loop, daemon=True)
            self.esp_thread.start()

        # 0.2초마다 제어 알고리즘 및 상태 업데이트
        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info("🚀 섀시 자동 제어 노드가 정상 가동되었습니다.")

    # ----------------------------------------------------
    # 3. 데이터 수신 콜백
    # ----------------------------------------------------
    def scan_callback(self, msg):
        """라이다에서 장애물 최소 거리 추출"""
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.min_obstacle_dist = min(valid_ranges)

    def odom_callback(self, msg):
        """SLAM/주행 오도메트리에서 속도 추출"""
        self.current_speed = abs(msg.twist.twist.linear.x)

    def read_esp32_loop(self):
        """ESP32 시리얼로부터 리프트 높이 데이터 읽기 (예: 'H:450.0' 형태)"""
        while rclpy.ok():
            if self.esp_ser and self.esp_ser.in_waiting > 0:
                try:
                    line = self.esp_ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("H:"): # ESP32에서 "H:높이값" 형태로 보낸다고 가정
                        self.lift_height = float(line.replace("H:", ""))
                except ValueError:
                    pass
            time.sleep(0.02)

    def check_limit_switch(self):
        """MKS 보드로 ? 보낸 후 리미트 스위치(Pn:) 감지"""
        if not hasattr(self, 'mks_ser') or not self.mks_ser:
            return False
        
        self.mks_ser.write(b"?\n")
        time.sleep(0.01)
        if self.mks_ser.in_waiting > 0:
            res = self.mks_ser.read(self.mks_ser.in_waiting).decode('utf-8', errors='ignore')
            if "Pn:" in res:
                return True # 완전히 수축됨 (스위치 눌림)
        return False

    # ----------------------------------------------------
    # 4. 핵심 제어 알고리즘 루프 (우선순위 판단)
    # ----------------------------------------------------
    def control_loop(self):
        self.is_limit_pressed = self.check_limit_switch()

        # [우선순위 1] 장애물이 0.4m 이내면 섀시 수축 (협소 구간)
        if self.min_obstacle_dist < 0.4:
            if not self.is_limit_pressed:
                # 스위치가 안 눌렸을 때만 수축 G-code 전송
                self.mks_ser.write(b"G1 X-5 Y-5 Z-5 F600\n")
                self.get_logger().info(f"⚠️ 장애물 감지({self.min_obstacle_dist:.2f}m) -> 섀시 수축 중...")
            else:
                self.get_logger().info("🔒 최대 수축 완료 (리미트 스위치 접촉)")

        # [우선순위 2] 리프트가 500mm 이상 올라갔으면 전복 방지 확장
        elif self.lift_height > 500.0:
            self.mks_ser.write(b"G1 X5 Y5 Z5 F600\n")
            self.get_logger().info(f"🏗️ 리프트 상승({self.lift_height:.1f}mm) -> 전복 방지 섀시 확장!")

        # [우선순위 3] 주행 속도가 0.5m/s 이상이면 확장
        elif self.current_speed > 0.5:
            self.mks_ser.write(b"G1 X5 Y5 Z5 F600\n")
            self.get_logger().info(f"💨 고속 주행({self.current_speed:.2f}m/s) -> 주행 안정성 섀시 확장!")

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedChassisController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()