#!/usr/bin/env python3
# =====================================================
# IMULIFT 리프트 조작 터미널 (터미널 A)
#
# 이 스크립트 하나가 시리얼 포트를 단독으로 열고 관리한다.
#   - H / M / T 입력 -> ESP32로 전송 -> 응답(DONE 등)을 이 화면에 출력
#   - BNO055 IMU 라인("<yaw,gyro_z>")은 이 화면에 찍지 않고
#     별도 로그 파일에 기록한다 -> 터미널 B에서 tail -f로 구독
#
# 터미널 A (이 스크립트):
#   python3 lift_control.py --port /dev/ttyACM0
#
# 터미널 B (같은 젯슨, SSH 새 세션):
#   tail -f /tmp/imulift_imu.log
#
# 시리얼 포트는 이 스크립트에서만 열기 때문에
# 두 프로세스가 같은 포트를 동시에 열어서 생기는 문제
# (한쪽 read()가 데이터를 가로채거나, 두 번째 open 시 보드가
#  리셋되는 문제 등)가 없다.
# =====================================================

import argparse
import re
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("[ERROR] pyserial이 설치되어 있지 않습니다. 설치: pip3 install pyserial")
    sys.exit(1)

IMU_PATTERN = re.compile(r"^<(-?\d+\.\d+),(-?\d+\.\d+)>$")


def ts():
    return time.strftime("%H:%M:%S")


def reader_loop(ser, imu_log_path, stop_flag):
    """
    시리얼에서 읽은 줄을 분류한다.
      - IMU 패턴 -> imu_log_path 파일에 기록 (터미널 B가 tail -f로 구독)
      - 그 외 (부팅 로그, DONE, 상태 메시지 등) -> 이 터미널(stdout)에 출력
    """
    with open(imu_log_path, "a", buffering=1) as imu_log:
        buf = b""
        while not stop_flag.is_set():
            try:
                chunk = ser.read(256)
            except serial.SerialException as e:
                print(f"\n[{ts()}] [ERROR] 시리얼 읽기 실패: {e}")
                stop_flag.set()
                break

            if not chunk:
                continue

            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue

                if IMU_PATTERN.match(line):
                    imu_log.write(f"[{ts()}] {line}\n")
                else:
                    print(f"[{ts()}] RX   {line}")


def main():
    ap = argparse.ArgumentParser(description="IMULIFT 리프트 조작 터미널 (H/M/T 입력용)")
    ap.add_argument("--port", default="/dev/ttyACM1",
                     help="시리얼 포트 (기본 /dev/ttyACM0, USB-UART 브리지 보드면 /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--imu-log", default="/tmp/imulift_imu.log",
                     help="IMU 라인을 기록할 파일 (터미널 B에서 tail -f)")
    args = ap.parse_args()

    # 시작할 때마다 IMU 로그 비우기 (이전 실행 기록 누적 방지)
    open(args.imu_log, "w").close()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        print(f"[ERROR] 시리얼 포트를 열 수 없습니다: {e}")
        print("  - 포트 확인: python3 -m serial.tools.list_ports")
        print("  - 권한 문제라면: sudo usermod -aG dialout $USER  (재로그인 필요)")
        sys.exit(1)

    print("=" * 60)
    print(f" IMULIFT 리프트 조작 터미널   port={args.port}  baud={args.baud}")
    print(f" IMU 로그 파일 : {args.imu_log}")
    print(f"   -> 다른 SSH 터미널에서:  tail -f {args.imu_log}")
    print("=" * 60)
    print("명령 입력: H (홈) / M (중간, 30000) / T (최상단, 75000)")
    print("종료: Ctrl+C 또는 'q' 입력")
    print("-" * 60)

    stop_flag = threading.Event()
    t = threading.Thread(target=reader_loop, args=(ser, args.imu_log, stop_flag), daemon=True)
    t.start()

    try:
        while not stop_flag.is_set():
            try:
                cmd = input()
            except EOFError:
                break

            cmd = cmd.strip()
            if cmd.lower() in ("q", "quit", "exit"):
                break
            if not cmd:
                continue

            print(f"[{ts()}] TX   {cmd}")
            ser.write((cmd + "\n").encode())
            ser.flush()

    except KeyboardInterrupt:
        pass
    finally:
        stop_flag.set()
        time.sleep(0.3)
        ser.close()
        print("\n종료.")


if __name__ == "__main__":
    main()