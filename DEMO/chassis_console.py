#!/usr/bin/env python3
# =====================================================
# 섀시 보드용 초간단 시리얼 콘솔 (screen/minicom 대용)
#
# 목적: 지금 붙어있는 보드가 진짜 Grbl인지, "?" 응답 포맷이
#       정확히 뭔지, $10/$5 설정이 뭔지를 직접 타이핑해서 눈으로 확인.
#
# 사용법:
#   python3 chassis_console.py --port /dev/ttyCH341USB0
#
# 콘솔에서 쳐볼 것들:
#   $I      -> Grbl 빌드 정보 (진짜 Grbl이면 "Grbl 1.1h ..." 식으로 응답)
#   $$      -> 전체 설정값 덤프 (여기서 $10=... , $5=... 줄을 확인)
#   ?       -> 실시간 상태 조회 (리미트 스위치를 손으로 누른 채 다시 입력해서
#              응답이 달라지는지 비교)
# =====================================================

import argparse
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("[ERROR] pyserial이 설치되어 있지 않습니다. 설치: pip3 install pyserial")
    sys.exit(1)


def ts():
    return time.strftime("%H:%M:%S")


def reader_loop(ser, stop_flag):
    buf = b""
    while not stop_flag.is_set():
        try:
            chunk = ser.read(256)
        except serial.SerialException as e:
            print(f"\n[{ts()}] [ERROR] 읽기 실패: {e}")
            stop_flag.set()
            break
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode(errors="replace").strip()
            if line:
                print(f"[{ts()}] RX   {line!r}")


def main():
    ap = argparse.ArgumentParser(description="섀시 보드 시리얼 콘솔")
    ap.add_argument("--port", default="/dev/ttyCH341USB0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        print(f"[ERROR] 포트를 열 수 없습니다: {e}")
        sys.exit(1)

    print("=" * 60)
    print(f" 섀시 콘솔  port={args.port}  baud={args.baud}")
    print(" 아무 명령이나 입력 후 Enter (예: $I / $$ / ? / G91)")
    print(" 종료: Ctrl+C 또는 'q'")
    print("=" * 60)

    stop_flag = threading.Event()
    t = threading.Thread(target=reader_loop, args=(ser, stop_flag), daemon=True)
    t.start()

    time.sleep(0.3)  # 부팅 배너 등 초기 출력 잡아내기

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
            print(f"[{ts()}] TX   {cmd!r}")
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