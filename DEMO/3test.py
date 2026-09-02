import serial
import time
import sys
import tty
import termios
import select

PORT = '/dev/ttyCH341USB0'
BAUD_RATE = 115200
STEPS_PER_MM = "666.67"

# ---------------- 3단계 목표 위치 (mm) ----------------
LEVEL_POS = {
    '1': 0.0,    # 완전 접힘
    '2': 27.5,   # 중간
    '3': 55.0,   # 완전 펼침 (최대, 이 이상 넘기지 말 것)
}
FEED_RATE = 700          # 1/2/3 단계 이동 속도
JOG_STEP_MM = 5.0        # 개별 축 조그 시 한 번에 이동하는 거리
JOG_FEED_RATE = 700      # 개별 축 조그 속도

PRE_EXTEND_MM = 10.0     # 홈잉 시작 전, 미리 펼쳐서 리미트 스위치에서 떨어뜨리는 거리
HOMING_STEP_MM = 8.0     # 홈잉 시 한 스텝당 이동거리
HOMING_FEED_RATE = 450   # 홈잉(수축) 속도
FINAL_PULL_MM = 10.0     # 스위치가 처음 눌린 순간부터 추가로 더 당기는 거리
FINAL_PULL_FEED = 600    # 10mm를 정확히 1초에 이동하도록 계산된 속도 (mm/min)
MAX_HOMING_DURATION = 8.0  # 이 시간(초) 안에 스위치가 안 눌리면 모터 보호를 위해 강제 중단


def get_key_nonblocking():
    """Linux(Jetson) 전용 비블로킹 키 입력"""
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


def send_and_wait_ok(ser, cmd, timeout=2.0):
    """명령을 보내고 GRBL이 'ok' 또는 'error'로 응답할 때까지 대기.
    버퍼 오버플로우로 인한 명령 씹힘을 방지하는 핵심 함수."""
    ser.reset_input_buffer()
    ser.write(cmd.encode())
    start = time.time()
    buf = ""
    while time.time() - start < timeout:
        if ser.in_waiting > 0:
            buf += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            if "ok" in buf or "error" in buf:
                return buf
        time.sleep(0.01)
    return buf


def wait_until_idle(ser, timeout=3.0):
    """현재 이동이 완전히 끝나서 GRBL이 Idle 상태가 될 때까지 대기."""
    start = time.time()
    while time.time() - start < timeout:
        ser.write(b"?\n")
        time.sleep(0.02)
        if ser.in_waiting > 0:
            res = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            if "Idle" in res:
                return True
    return False


def get_pressed_axes(ser):
    """GRBL 상태조회 응답에서 'Pn:' 뒤에 오는 축 문자(X/Y/Z)를 파싱해서
    현재 눌려있는 리미트 스위치 축 집합을 반환."""
    ser.write(b"?\n")
    time.sleep(0.015)
    if ser.in_waiting > 0:
        res = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        idx = res.find("Pn:")
        if idx != -1:
            rest = res[idx + 3:]
            end = min([p for p in (rest.find("|"), rest.find(">")) if p != -1], default=len(rest))
            pins = rest[:end]
            return set(ch for ch in pins if ch in ("X", "Y", "Z"))
    return set()


def home_chassis(ser):
    """[0]을 누를 때마다 언제든 다시 실행되는 홈잉 루틴.
    먼저 PRE_EXTEND_MM 만큼 펼쳐서 스위치에서 떨어뜨린 후 수축을 시작한다.
    리미트 스위치가 '단 하나라도' 눌리는 순간 -> 전체가 다 닫힌 것으로 간주하고,
    FINAL_PULL_MM 만큼 1초간 추가로 더 당긴 뒤 즉시 종료한다."""
    print(f"\n🏠 홈잉 시작: 리미트 스위치에서 떨어지도록 먼저 {PRE_EXTEND_MM}mm 펼칩니다...")
    send_and_wait_ok(ser, "G91\n")
    pre_extend_cmd = f"G1 X{PRE_EXTEND_MM} Y{PRE_EXTEND_MM} Z{PRE_EXTEND_MM} F{HOMING_FEED_RATE}\n"
    send_and_wait_ok(ser, pre_extend_cmd)
    wait_until_idle(ser, timeout=(PRE_EXTEND_MM / HOMING_FEED_RATE) * 60.0 + 2.0)

    print("영점 조절: 리미트 스위치가 하나라도 눌릴 때까지 수축합니다...")

    homing_start = time.time()
    switch_found = False

    while True:
        if time.time() - homing_start > MAX_HOMING_DURATION:
            print(f"🛑 {MAX_HOMING_DURATION}초 안에 스위치가 감지되지 않았습니다. 모터 보호를 위해 홈잉을 중단합니다.")
            print("   -> 리미트 스위치 배선/장착 상태를 확인해주세요.")
            break

        pressed = get_pressed_axes(ser)

        if len(pressed) > 0:
            print(f"⚠️ 스위치 감지됨 ({sorted(pressed)}). {FINAL_PULL_MM}mm 추가로 더 당기고 정지합니다...")
            final_cmd = f"G1 X-{FINAL_PULL_MM} Y-{FINAL_PULL_MM} Z-{FINAL_PULL_MM} F{FINAL_PULL_FEED}\n"
            send_and_wait_ok(ser, final_cmd)
            wait_until_idle(ser, timeout=2.0)
            switch_found = True
            break

        cmd = f"G1 X-{HOMING_STEP_MM} Y-{HOMING_STEP_MM} Z-{HOMING_STEP_MM} F{HOMING_FEED_RATE}\n"
        send_and_wait_ok(ser, cmd)
        wait_until_idle(ser, timeout=1.5)

    if not switch_found:
        print("⚠️ 영점을 확실히 잡지 못했습니다. 1/2/3 단계 이동값이 부정확할 수 있습니다.\n")
        return False

    time.sleep(0.2)
    send_and_wait_ok(ser, "G92 X0 Y0 Z0\n")  # 현재 지점을 0으로 선언
    send_and_wait_ok(ser, "G90\n")           # 이후 절대좌표 모드로 전환
    print("✅ 영점 조절 완료. 0mm = 완전 접힘 상태로 설정됨.\n")
    return True


def set_zero_here(ser):
    """수동으로 WSAD/RF 조그를 통해 원하는 지점(예: 완전 접힘)에 맞춘 뒤,
    자동 홈잉(리미트 스위치 탐색) 없이 그 현재 위치를 그대로 0점으로 선언한다."""
    print("\n📍 현재 위치를 0mm(영점)으로 설정합니다...")
    send_and_wait_ok(ser, "G92 X0 Y0 Z0\n")
    send_and_wait_ok(ser, "G90\n")
    print("✅ 수동 영점 설정 완료. 현재 위치 = 0mm로 등록됨.\n")


def run_interactive(ser):
    homed = False
    current_level = None

    print("==========================================")
    print("🎮 섀시 제어 테스트 모드")
    print("==========================================")
    print(" [0]     : 자동 홈잉 실행 (리미트 스위치까지 수축, 언제든 재실행 가능)")
    print(" [Z]     : 수동 영점 설정 - 현재 위치를 그대로 0mm로 등록")
    print(" [1/2/3] : 0mm / 27.5mm / 55mm 이동 (영점 설정 후에만 동작)")
    print(" [W/S]   : X축 개별 +/- 조그")
    print(" [D/A]   : Y축 개별 +/- 조그 (D=확장 / A=수축, 배선 반전 보정됨)")
    print(" [R/F]   : Z축 개별 +/- 조그")
    print(" [ESC/Ctrl+C] : 종료")
    print("==========================================\n")
    print("WSAD/RF로 위치를 먼저 조정한 뒤 [Z]로 영점 설정하거나, [0]으로 자동 홈잉하세요.")

    while True:
        raw_key = get_key_nonblocking()
        if raw_key is None:
            continue

        if raw_key == '\x1b' or raw_key == '\x03':
            print("\n👋 종료합니다.")
            break

        key = raw_key.lower()

        if raw_key == '0':
            homed = home_chassis(ser)
            if homed:
                current_level = '1'
            continue

        if key == 'z':
            set_zero_here(ser)
            homed = True
            current_level = '1'
            continue

        if raw_key in LEVEL_POS:
            if not homed:
                print("\r⚠️ 아직 영점이 안 잡혔습니다. [0](자동 홈잉) 또는 [Z](수동 영점)를 먼저 사용하세요.            ", end="", flush=True)
                continue

            target = LEVEL_POS[raw_key]
            cmd = f"G90 G1 X{target:.2f} Y{target:.2f} Z{target:.2f} F{FEED_RATE}\n"
            ser.write(cmd.encode())
            current_level = raw_key
            print(f"\r▶ {raw_key}단계 이동 -> {target}mm 명령 전송              ", end="", flush=True)

        elif key in ('w', 's', 'd', 'a', 'r', 'f'):
            jog_cmd = ""
            if key == 'w':
                jog_cmd = f"G91 G1 X{JOG_STEP_MM} F{JOG_FEED_RATE}\n"
            elif key == 's':
                jog_cmd = f"G91 G1 X-{JOG_STEP_MM} F{JOG_FEED_RATE}\n"
            # ⚠️ D/A만 하드웨어 배선이 반대로 꽂혀있어서, 여기서 매핑만 서로 바꿔줌
            elif key == 'd':
                jog_cmd = f"G91 G1 Y-{JOG_STEP_MM} F{JOG_FEED_RATE}\n"
            elif key == 'a':
                jog_cmd = f"G91 G1 Y{JOG_STEP_MM} F{JOG_FEED_RATE}\n"
            elif key == 'r':
                jog_cmd = f"G91 G1 Z{JOG_STEP_MM} F{JOG_FEED_RATE}\n"
            elif key == 'f':
                jog_cmd = f"G91 G1 Z-{JOG_STEP_MM} F{JOG_FEED_RATE}\n"

            ser.write(jog_cmd.encode())
            print(f"\r▶ 조그 전송: {jog_cmd.strip()}              ", end="", flush=True)


def main():
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        time.sleep(2)

        # 🛠️ 하드웨어 세팅 (홈잉 여부와 무관하게 시작 시 항상 적용)
        ser.write(b"$1=25\n")
        ser.write(b"$20=0\n")              # 소프트 리미트 OFF
        ser.write(b"$21=0\n")              # 하드웨어 강제 알람 OFF
        ser.write(f"$100={STEPS_PER_MM}\n".encode())
        ser.write(f"$101={STEPS_PER_MM}\n".encode())
        ser.write(f"$102={STEPS_PER_MM}\n".encode())
        ser.write(b"$130=9999.000\n")
        ser.write(b"$131=9999.000\n")
        ser.write(b"$132=9999.000\n")
        ser.write(b"G91\n")  # 초기 모드: 상대좌표 (홈잉 전 자유 조그를 위함)
        time.sleep(0.5)

        run_interactive(ser)

    except Exception as e:
        print(f"\n❌ 에러 발생: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == '__main__':
    main()