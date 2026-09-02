import serial
import time
import sys
import tty
import termios
import select

PORT = '/dev/ttyCH341USB2'
BAUD_RATE = 115200
STEPS_PER_MM = "666.67"

def get_key_nonblocking():
    """키보드 입력을 기다리지 않고 실시간으로 가져오는 비블로킹 함수"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.005)
        if rlist:
            return sys.stdin.read(1)
        else:
            return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def get_limit_status(ser):
    """현재 리미트 스위치가 눌려있는지 여부를 반환하는 함수"""
    ser.write(b"?\n")
    time.sleep(0.005)
    if ser.in_waiting > 0:
        res = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        if "Pn:" in res:
            return True, res.strip()
    return False, ""

def main():
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        
        # 🛠️ 하드웨어 세팅 (알람 락 완전 해제)
        ser.write(b"$1=25\n")               
        ser.write(b"$20=0\n")              # 소프트웨어 리미트 끄기
        ser.write(b"$21=0\n")              # 하드웨어 강제 알람 끄기
        ser.write(f"$100={STEPS_PER_MM}\n".encode())
        ser.write(f"$101={STEPS_PER_MM}\n".encode())
        ser.write(f"$102={STEPS_PER_MM}\n".encode())
        
        ser.write(b"$130=9999.000\n")
        ser.write(b"$131=9999.000\n")
        ser.write(b"$132=9999.000\n")
        
        ser.write(b"G91\n")                # 상대 이동 모드
        
        step_distance = 5.0 
        feed_rate = 600     
        
        print("\n==========================================")
        print("🎮 MKS DLC32 스마트 방향 차단 모드")
        print("==========================================")
        print(" [W/D/R] : 섀시 확장 (+) (스위치 눌려도 언제든 가능)")
        print(" [S/A/F] : 섀시 수축 (-) (스위치 누르면 자동 차단)")
        print(" [ 1~4 ] : 이동거리 변경 | [ +/- ] : 속도 변경")
        print(" [ESC/Ctrl+C] : 종료")
        print("==========================================\n")

        limit_notified = False

        while True:
            # 1. 리미트 스위치 상태 확인
            is_pressed, status_msg = get_limit_status(ser)

            if is_pressed and not limit_notified:
                print(f"\n🔒 [완전 닫힘 감지] 수축 키(S/A/F)는 차단됩니다. 확장 키(W/D/R)를 눌러 펼쳐주세요.")
                limit_notified = True
            elif not is_pressed and limit_notified:
                print(f"\n🔓 [스위치 해제] 정상 상태로 복귀했습니다. 모든 방향 이동이 가능합니다.")
                limit_notified = False

            # 2. 키보드 입력 체크
            raw_key = get_key_nonblocking()
            if raw_key is None:
                continue
                
            key = raw_key.lower()
            
            if key == '\x1b' or key == '\x03':
                print("\n👋 키보드 제어를 종료합니다.")
                break
                
            elif key == '1': step_distance = 1.0; print(f"\r📏 이동 거리 설정: {step_distance}mm", end="")
            elif key == '2': step_distance = 5.0; print(f"\r📏 이동 거리 설정: {step_distance}mm", end="")
            elif key == '3': step_distance = 10.0; print(f"\r📏 이동 거리 설정: {step_distance}mm", end="")
            elif key == '4': step_distance = 50.0; print(f"\r📏 이동 거리 설정: {step_distance}mm", end="")
            
            elif key == '+' or key == '=': feed_rate += 100; print(f"\r🚀 이동 속도 설정: F{feed_rate}", end="")
            elif key == '-': feed_rate = max(100, feed_rate - 100); print(f"\r🚀 이동 속도 설정: F{feed_rate}", end="")

            # 3. 방향에 따른 이동 조작
            cmd = ""
            
            # 🟢 [확장 방향 (+)]: 언제든지 가능!
            if key == 'w': cmd = f"G1 X{step_distance} F{feed_rate}\n"
            elif key == 'd': cmd = f"G1 Y{step_distance} F{feed_rate}\n"
            elif key == 'r': cmd = f"G1 Z{step_distance} F{feed_rate}\n"
            
            # 🔴 [수축 방향 (-)]: 리미트 스위치가 눌려있으면 차단!
            elif key == 's':
                if is_pressed: print("\r⚠️ [수축 차단] 이미 완전히 닫혀있습니다!      ", end="")
                else: cmd = f"G1 X-{step_distance} F{feed_rate}\n"
            elif key == 'a':
                if is_pressed: print("\r⚠️ [수축 차단] 이미 완전히 닫혀있습니다!      ", end="")
                else: cmd = f"G1 Y-{step_distance} F{feed_rate}\n"
            elif key == 'f':
                if is_pressed: print("\r⚠️ [수축 차단] 이미 완전히 닫혀있습니다!      ", end="")
                else: cmd = f"G1 Z-{step_distance} F{feed_rate}\n"

            if cmd:
                ser.write(cmd.encode())
                print(f"\r▶ 전송: {cmd.strip()}       ", end="")

    except Exception as e:
        print(f"\n❌ 에러 발생: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
