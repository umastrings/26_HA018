#!/usr/bin/env python3
"""care_voice_node: /care/state 상태에 맞춰 mp3 안내 음성을 재생한다.

상태 값: IDLE / PATROL / BEDSORE / EMERGENCY / ESTOP (I / P / E / S 한 글자도 허용)
- PATROL    : patrol.mp3 1회
- EMERGENCY : alarm → fall_1 → fall_119 → fall_aed 순서로, 상태가 바뀔 때까지 반복
- BEDSORE   : alarm → bedsore 1회 (bedsore.mp3가 없으면 alarm만)
- ESTOP     : estop.mp3 1회
- IDLE      : 재생 중인 소리 정지. EMERGENCY/ESTOP/BEDSORE에서 IDLE로 풀리면 clear.mp3 1회

필요: sudo apt install mpg123
테스트: ros2 topic pub --once /care/state std_msgs/msg/String "{data: EMERGENCY}"
"""
import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ALIAS = {'I': 'IDLE', 'P': 'PATROL', 'E': 'EMERGENCY', 'S': 'ESTOP'}


class CareVoiceNode(Node):
    def __init__(self):
        super().__init__('care_voice')
        self.declare_parameter('sound_dir', os.path.expanduser('~/sounds'))
        # aplay -l 로 확인한 사운드카드 이름. 예: plughw:Device,0 또는 plughw:1,0
        self.declare_parameter('device', 'plughw:Audio,0')
        self.declare_parameter('patrol_file', 'patrol.mp3')
        self.declare_parameter('emergency_files',
                               ['alarm.mp3', 'fall_1.mp3', 'fall_119.mp3', 'fall_aed.mp3'])
        self.declare_parameter('bedsore_files', ['alarm.mp3', 'bedsore.mp3'])
        self.declare_parameter('estop_file', 'estop.mp3')
        self.declare_parameter('clear_file', 'clear.mp3')
        self.declare_parameter('emergency_repeat', True)

        self.state = None
        self.gen = 0            # 상태가 바뀔 때마다 증가 → 이전 재생 스레드 종료
        self.proc = None
        self.lock = threading.Lock()

        self.create_subscription(String, '/care/state', self.on_state, 10)
        self.check_files()
        self.get_logger().info('care_voice 준비 완료, /care/state 대기 중')

    def p(self, name):
        return self.get_parameter(name).value

    def path(self, f):
        return os.path.join(self.p('sound_dir'), f)

    def check_files(self):
        files = [self.p('patrol_file'), self.p('estop_file'), self.p('clear_file')] \
            + list(self.p('emergency_files')) + list(self.p('bedsore_files'))
        for f in files:
            if f and not os.path.isfile(self.path(f)):
                self.get_logger().warn(f'파일 없음: {self.path(f)}')

    def playlist_for(self, state, prev=None):
        if state == 'IDLE' and prev in ('EMERGENCY', 'ESTOP', 'BEDSORE'):
            return [self.p('clear_file')], False  # 상황 해제 안내
        if state == 'BEDSORE':
            return list(self.p('bedsore_files')), False
        if state == 'PATROL':
            return [self.p('patrol_file')], False
        if state == 'EMERGENCY':
            return list(self.p('emergency_files')), bool(self.p('emergency_repeat'))
        if state == 'ESTOP':
            return [self.p('estop_file')], False
        return [], False  # IDLE 또는 알 수 없는 상태 → 무음

    def on_state(self, msg):
        s = msg.data.strip().upper()
        s = ALIAS.get(s, s)
        if s == self.state:
            return
        self.get_logger().info(f'상태 {self.state} → {s}')
        prev, self.state = self.state, s
        self.stop()
        playlist, loop = self.playlist_for(s, prev)
        playlist = [f for f in playlist if f]
        if playlist:
            threading.Thread(target=self.run, args=(self.gen, playlist, loop),
                             daemon=True).start()

    def run(self, gen, playlist, loop):
        while True:
            for f in playlist:
                path = self.path(f)
                if not os.path.isfile(path):
                    continue
                for attempt in range(3):  # 장치가 아직 사용 중이면 잠깐 기다렸다 다시 시도
                    with self.lock:
                        if gen != self.gen:
                            return
                        self.proc = subprocess.Popen(
                            ['mpg123', '-q', '-o', 'alsa', '-a', self.p('device'), path],
                            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
                        proc = self.proc
                    proc.wait()
                    if proc.returncode in (0, -15, -9) or gen != self.gen:
                        break
                    err = proc.stderr.read().decode(errors='ignore').strip()
                    if attempt < 2:
                        time.sleep(0.3)
                        continue
                    self.get_logger().error(f'재생 실패 ({f}): {err}')
                    return
            if not loop:
                return

    def stop(self):
        with self.lock:
            self.gen += 1
            proc, self.proc = self.proc, None
        if proc and proc.poll() is None:
            proc.terminate()
            # 사운드카드(plughw)는 한 번에 한 프로그램만 쓸 수 있어서,
            # 이전 재생이 완전히 끝나 장치를 놓을 때까지 기다린다.
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

    def destroy_node(self):
        self.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = CareVoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
