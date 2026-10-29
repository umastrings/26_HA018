#!/usr/bin/env python3
"""care_led_bridge: /care/state 를 LED 나노에 시리얼로 전달하고, 나노의 비상정지 버튼 상태를 ROS로 올린다.

젯슨 → 나노 : <L,I> <L,P> <L,E> <L,S>   (상태가 바뀌면 즉시 + 1초마다 반복 전송)
              나노는 3초간 명령이 없으면 노랑 깜빡임(통신 끊김)이므로 반복 전송이 필요하다.
              BEDSORE(욕창 위험)는 나노 펌웨어 수정 없이 E(빨강 깜빡임)로 보낸다.
나노 → 젯슨 : <X,1> / <X,0>  → /care/estop_hw (std_msgs/Bool) 발행
              LED_NANO READY → 로그 출력

필요: pyserial (pip3 install pyserial)
실행: python3 care_led_bridge_node.py --ros-args -p port:=/dev/ttyCH341USB0
      ※ port는 꼭 지정. UNO도 같은 칩이라 잘못 열면 UNO가 리셋된다.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

import serial

STATE_TO_LETTER = {
    'IDLE': 'I', 'PATROL': 'P', 'EMERGENCY': 'E', 'ESTOP': 'S', 'BEDSORE': 'E',
    'I': 'I', 'P': 'P', 'E': 'E', 'S': 'S',
}


class CareLedBridge(Node):
    def __init__(self):
        super().__init__('care_led_bridge')
        self.declare_parameter('port', '')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('resend_period', 1.0)

        self.port = self.get_parameter('port').value
        if not self.port:
            raise SystemExit('port를 지정하세요. 예: --ros-args -p port:=/dev/ttyCH341USB0')

        self.letter = 'I'          # 처음엔 대기(초록)
        self.ser = None
        self.lock = threading.Lock()
        self.last_x_time = None    # 나노가 마지막으로 <X,..>를 보낸 시각

        self.pub_estop = self.create_publisher(Bool, '/care/estop_hw', 10)
        self.create_subscription(String, '/care/state', self.on_state, 10)
        self.create_timer(float(self.get_parameter('resend_period').value), self.resend)

        self.running = True
        threading.Thread(target=self.reader, daemon=True).start()
        self.get_logger().info(f'LED 연결 노드 시작 (포트 {self.port})')

    # ---------- 시리얼 ----------
    def open_serial(self):
        try:
            s = serial.Serial(self.port, int(self.get_parameter('baud').value), timeout=0.2)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'나노 연결 실패 ({self.port}): {e} → 2초 후 재시도',
                                   throttle_duration_sec=10.0)
            return None
        time.sleep(2.0)            # 포트를 열면 나노가 리셋되므로 부팅을 기다린다
        self.get_logger().info('나노 연결됨')
        return s

    def send(self, letter):
        with self.lock:
            if self.ser is None:
                return
            try:
                self.ser.write(f'<L,{letter}>\n'.encode())
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'나노 전송 실패: {e}')
                self._close()

    def _close(self):          # lock 잡은 상태에서 호출
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    # ---------- ROS ----------
    def on_state(self, msg):
        letter = STATE_TO_LETTER.get(msg.data.strip().upper())
        if letter is None:
            self.get_logger().warn(f'알 수 없는 상태: {msg.data}')
            return
        if letter != self.letter:
            self.get_logger().info(f'LED {self.letter} → {letter} ({msg.data})')
            self.letter = letter
            self.send(letter)      # 바뀌었을 때만 즉시 전송, 나머지는 1초 타이머가 담당

    def resend(self):
        self.send(self.letter)
        if self.last_x_time and time.time() - self.last_x_time > 3.0:
            self.get_logger().warn('나노에서 버튼 상태가 3초 넘게 안 옴 (USB 확인)',
                                   throttle_duration_sec=10.0)

    # ---------- 나노 → 젯슨 ----------
    def handle_line(self, line):
        if line == '<X,1>':
            self.last_x_time = time.time()
            self.pub_estop.publish(Bool(data=True))
        elif line == '<X,0>':
            self.last_x_time = time.time()
            self.pub_estop.publish(Bool(data=False))
        elif 'LED_NANO READY' in line:
            self.get_logger().info('나노 부팅 확인: LED_NANO READY')

    def reader(self):
        buf = b''
        while self.running and rclpy.ok():
            if self.ser is None:
                s = self.open_serial()
                if s is None:
                    time.sleep(2.0)
                    continue
                with self.lock:
                    self.ser = s
                buf = b''
                self.send(self.letter)
            try:
                ser = self.ser
                chunk = ser.read(64) if ser else b''
            except (serial.SerialException, OSError, AttributeError, TypeError) as e:
                self.get_logger().error(f'나노 수신 실패: {e} → 재연결')
                with self.lock:
                    self._close()
                continue
            buf += chunk
            # 줄 단위로 끊어서 처리 (한 줄이 두 번에 나눠 와도 안전)
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                line = raw.decode(errors='ignore').strip()
                if line:
                    self.handle_line(line)
            if len(buf) > 256:
                buf = b''

    def destroy_node(self):
        self.running = False
        with self.lock:
            self._close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CareLedBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
