import time
import serial

# ======================
# Serial Setup
# ======================
try:
    ser = serial.Serial("COM3", 115200, timeout=0.1)
    print("Serial connected.")
except Exception as e:
    print("Serial connection failed. Commands will only be printed to terminal.")
    ser = None

# ======================
# 설정 및 변수
# ======================
TARGET_ANGLE = 60.0      # 목표 각도 (60°)
TOLERANCE = 10.0         # 허용 오차 (±10°)

def get_motor_commands(measured_angle):
    """
    측정 각도를 입력받아 TARGET_ANGLE ± TOLERANCE 범위 내이면 STOP,
    범위 미만이면 REV, 초과이면 FWD 명령을 결정하고, 오차를 반환합니다.
    """
    if TARGET_ANGLE - TOLERANCE <= measured_angle <= TARGET_ANGLE + TOLERANCE:
        return "STOP", 0.0
    elif measured_angle < TARGET_ANGLE - TOLERANCE:
        return "REV", TARGET_ANGLE - measured_angle
    else:  # measured_angle > TARGET_ANGLE + TOLERANCE
        return "FWD", measured_angle - TARGET_ANGLE

while True:
    print("\n각 모터의 측정 각도를 입력하세요.")
    left_angles = []
    right_angles = []
    
    # 좌측 5개, 우측 5개 모터 각도 입력 받기
    for i in range(1, 6):
        try:
            angle = float(input(f"Motor L{i} 측정 각도: "))
            left_angles.append(angle)
        except ValueError:
            print("숫자 형식으로 입력해주세요.")
            break
        try:
            angle = float(input(f"Motor R{i} 측정 각도: "))
            right_angles.append(angle)
        except ValueError:
            print("숫자 형식으로 입력해주세요.")
            break

    if len(left_angles) != 5 or len(right_angles) != 5:
        print("모든 모터에 대해 5개의 값을 입력하지 않았습니다. 다시 시도합니다.")
        continue

    # 각 모터별 명령 및 오차 계산
    commands_left = []
    errors_left = []
    commands_right = []
    errors_right = []

    for i in range(5):
        cmd, err = get_motor_commands(left_angles[i])
        commands_left.append(cmd)
        errors_left.append(err)
        cmd, err = get_motor_commands(right_angles[i])
        commands_right.append(cmd)
        errors_right.append(err)

    # 각 모터의 CSV 문자열을 개별 줄로 생성
    debug_lines = []
    for i in range(5):
        left_line = f"L{i+1},{commands_left[i]},{errors_left[i]:.2f}"
        right_line = f"R{i+1},{commands_right[i]},{errors_right[i]:.2f}"
        debug_lines.append(left_line)
        debug_lines.append(right_line)
    
    debug_csv = "\n".join(debug_lines)
    
    # CSV 형식의 디버깅 메시지를 터미널에 출력
    print("\nCSV 형식 디버깅 메시지:")
    print(debug_csv)

    # 시리얼 포트로도 전송
    if ser is not None:
        ser.write((debug_csv + "\n").encode())

    cont = input("\n계속 테스트 하시겠습니까? (y/n): ")
    if cont.lower() != "y":
        break

print("실험을 종료합니다.")
