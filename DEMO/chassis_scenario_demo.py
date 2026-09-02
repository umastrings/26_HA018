import serial
import time

PORT = '/dev/ttyCH341USB1'
BAUD_RATE = 115200
STEPS_PER_MM = "666.67"

# ---------------- 3단계 목표 위치 (mm) ----------------
LEVEL_POS = {
    0: 0.0,    # 완전 접힘
    1: 27.5,   # 중간
    2: 55.0,   # 완전 펼침 (최대, 이 이상 넘기지 말 것)
}
FEED_RATE = 650
HOMING_STEP_MM = 2.0
HOMING_FEED_RATE = 300

# ==================================================================
# 🎬 시나리오 정의 (촬영 콘티에 맞게 이 부분만 자유롭게 수정하세요)
#   name        : 터미널에 표시할 상황 설명 (촬영 시 나레이션과 맞추면 좋음)
#   level       : 이 단계에서 이동할 목표 단계 (0/1/2, LEVEL_POS 참조)
#   wait_before : 이 동작을 실행하기 전 대기 시간(초) - 앞 장면 촬영 시간 확보용
# ==================================================================
SCENARIO = [
    {"name": "대기 상태 - 평상시 주행 폭",              "level": 2, "wait_before": 3},
    {"name": "전방 협소 통로 진입 감지 -> 섀시 접힘",     "level": 0, "wait_before": 5},
    {"name": "통로 통과 완료 -> 평상시 주행 폭으로 복귀", "level": 2, "wait_before": 5},
    {"name": "리프트 상승 감지 -> 전복 방지 최대 확장",   "level": 2, "wait_before": 5},
    {"name": "동작 종료 -> 안전을 위해 완전 접힘",        "level": 0, "wait_before": 5},
]
# ==================================================================


def is_limit_pressed(ser):
    ser.write(b"?\n")
    time.sleep(0.02)
    if ser.in_waiting > 0:
        res = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        if "Pn:" in res:
            return True
    return False


def home_chassis(ser):
    """리미트 스위치 눌릴 때까지 수축 -> 그 지점을 0mm로 선언"""
    print("영점 조절: 리미트 스위치 눌릴 때까지 수축합니다...")
    ser.write(b"G91\n")

    while True:
        if is_limit_pressed(ser):
            break
        cmd = f"G1 X-{HOMING_STEP_MM} Y-{HOMING_STEP_MM} Z-{HOMING_STEP_MM} F{HOMING_FEED_RATE}\n"
        ser.write(cmd.encode())
        time.sleep(0.3)

    time.sleep(0.3)
    ser.write(b"G92 X0 Y0 Z0\n")
    ser.write(b"G90\n")
    print("✅ 영점 조절 완료. 0mm = 완전 접힘 상태로 설정됨.\n")


def move_to_level(ser, level):
    target = LEVEL_POS[level]
    cmd = f"G1 X{target:.2f} Y{target:.2f} Z{target:.2f} F{FEED_RATE}\n"
    ser.write(cmd.encode())
    print(f"▶ {level}단계 이동 명령 전송 -> {target}mm")


def main():
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        time.sleep(2)

        # 하드웨어 세팅
        ser.write(b"$1=25\n")
        ser.write(b"$20=0\n")
        ser.write(b"$21=0\n")
        ser.write(f"$100={STEPS_PER_MM}\n".encode())
        ser.write(f"$101={STEPS_PER_MM}\n".encode())
        ser.write(f"$102={STEPS_PER_MM}\n".encode())
        ser.write(b"$130=9999.000\n")
        ser.write(b"$131=9999.000\n")
        ser.write(b"$132=9999.000\n")
        time.sleep(0.5)

        home_chassis(ser)

        print("==========================================")
        print("🎬 무선 데모 시나리오 자동 재생 모드")
        print("==========================================")
        print(f" 총 {len(SCENARIO)}단계 시나리오가 자동으로 순서대로 실행됩니다.")
        print(" 키보드 입력 없이 타이머로 진행됩니다. 중단하려면 Ctrl+C.")
        print("==========================================\n")

        for i, step in enumerate(SCENARIO, start=1):
            print(f"[{i}/{len(SCENARIO)}] 다음 상황까지 {step['wait_before']}초 대기: \"{step['name']}\"")
            time.sleep(step['wait_before'])
            print(f"  -> 실행: {step['name']}")
            move_to_level(ser, step['level'])
            print()  # 가독성을 위한 줄바꿈

        print("🎉 시나리오 재생이 모두 끝났습니다.")

    except KeyboardInterrupt:
        print("\n👋 사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"\n❌ 에러 발생: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == '__main__':
    main()