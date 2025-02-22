import cv2
import numpy as np
import time
import serial

# ======================
# Serial Setup
# ======================
try:
    ser = serial.Serial("COM3", 115200, timeout=0.1)
    print("Serial connected.")
except Exception as e:
    print("Serial connection failed, using keyboard input for done signals.")
    ser = None

# ======================
# Helper functions
# ======================
def detect_black_blobs(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 0  # 검정색
    params.filterByArea = True
    params.minArea = 50
    params.maxArea = 5000
    params.filterByCircularity = True
    params.minCircularity = 0.7
    params.filterByConvexity = False
    params.filterByInertia = False
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(gray)
    points = [(int(kp.pt[0]), int(kp.pt[1])) for kp in keypoints]
    return points

def detect_outer_object_contour(image):
    h, w = image.shape[:2]
    margin = int(0.1 * w)
    cropped = image[:, margin:w-margin]
    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5,5), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255,
                                   cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=lambda c: cv2.contourArea(c))
        outer_contour = cv2.convexHull(largest)
        outer_contour += np.array([[[margin, 0]]])
        return outer_contour
    return None

def line_intersection(line1, line2):
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2
    denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if denom == 0:
        return None
    intersect_x = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)) / denom
    intersect_y = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)) / denom
    return (int(intersect_x), int(intersect_y))

def is_point_on_segment(pt, segment, tol=1e-6):
    (x, y) = pt
    (x1, y1), (x2, y2) = segment
    return (min(x1, x2)-tol <= x <= max(x1, x2)+tol) and (min(y1, y2)-tol <= y <= max(y1, y2)+tol)

def get_intersections_for_line(line, contour):
    intersections = []
    pts = contour[:,0,:]
    for i in range(len(pts)):
        seg = (tuple(pts[i]), tuple(pts[(i+1)%len(pts)]))
        inter_pt = line_intersection(line, seg)
        if inter_pt is not None:
            if is_point_on_segment(inter_pt, line) and is_point_on_segment(inter_pt, seg):
                intersections.append(inter_pt)
    return intersections

def trimmed_mean(data, trim_fraction=0.1):
    n = len(data)
    if n == 0:
        return 0
    sorted_data = sorted(data)
    trim_count = int(n * trim_fraction)
    if n - 2*trim_count <= 0:
        return sum(sorted_data)/n
    trimmed_data = sorted_data[trim_count:n-trim_count]
    return sum(trimmed_data)/len(trimmed_data)

def compute_relative(baseline_angles):
    valid = [a for a in baseline_angles if a > 0]
    if not valid:
        return baseline_angles
    base = min(valid)
    return [a - base if a > 0 else 0 for a in baseline_angles]

# ==============================
# 설정 및 변수
# ==============================
CALIBRATION_DURATION = 5.0  # 캘리브레이션 시간 (초)
calibration_start_time = time.time()
calibrated = False

left_angle_data = [[] for _ in range(5)]
right_angle_data = [[] for _ in range(5)]
calibration_left_baseline = [0] * 5
calibration_right_baseline = [0] * 5

TARGET_ANGLE = 100.0
TOLERANCE = 10.0  # 즉, 90° ~ 110° 범위

# 캘리브레이션 동안 각 라인의 교점 좌표(좌측, 우측)를 저장할 리스트 (각각 5개)
left_intersections_calib = [[] for _ in range(5)]
right_intersections_calib = [[] for _ in range(5)]
# 캘리브레이션 후 고정된 교점 좌표 (각각 5개)
fixed_intersections_left = [None] * 5
fixed_intersections_right = [None] * 5

# 전역 done 신호: "0" 입력 시 모든 모터에 대해 done 처리
motor_done_left = [False] * 5    
motor_done_right = [False] * 5   

# ==============================
# 메인 루프
# ==============================
cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 영상을 가져올 수 없습니다.")
        break

    frame_display = frame.copy()
    frame_width = frame.shape[1]

    # -------- 시리얼 데이터 읽기 --------
    if ser is not None and ser.in_waiting:
        line = ser.readline().decode().strip()
        if line == "0":
            for i in range(5):
                motor_done_left[i] = True
                motor_done_right[i] = True
            print("Received global done signal via serial (0)")
    # ------------------------------------

    # 외형선 검출 (캘리브레이션 동안 사용)
    outer_contour = detect_outer_object_contour(frame)

    # 캘리브레이션 중이면, 교점 좌표 저장
    if not calibrated and outer_contour is not None:
        points = detect_black_blobs(frame)
        if len(points) == 10:
            sorted_by_x = sorted(points, key=lambda p: p[0])
            left_points = sorted_by_x[:5]
            right_points = sorted_by_x[5:]
            left_points = sorted(left_points, key=lambda p: p[1])
            right_points = sorted(right_points, key=lambda p: p[1])
            for idx in range(5):
                line_temp = (left_points[idx], right_points[idx])
                intersections = get_intersections_for_line(line_temp, outer_contour)
                if intersections:
                    intersections = sorted(intersections, key=lambda pt: pt[0])
                    left_int = intersections[0]
                    right_int = intersections[-1]
                    left_intersections_calib[idx].append(left_int)
                    right_intersections_calib[idx].append(right_int)

    # 캘리브레이션 완료 후 고정된 교점 사용
    if calibrated:
        used_fixed_left = fixed_intersections_left
        used_fixed_right = fixed_intersections_right
    else:
        # 캘리브레이션 중에는 임시로 외형선에서 교점을 계산
        used_fixed_left = [None] * 5
        used_fixed_right = [None] * 5
        if outer_contour is not None:
            points = detect_black_blobs(frame)
            if len(points) == 10:
                sorted_by_x = sorted(points, key=lambda p: p[0])
                left_points = sorted_by_x[:5]
                right_points = sorted_by_x[5:]
                left_points = sorted(left_points, key=lambda p: p[1])
                right_points = sorted(right_points, key=lambda p: p[1])
                for idx in range(5):
                    line_temp = (left_points[idx], right_points[idx])
                    intersections = get_intersections_for_line(line_temp, outer_contour)
                    if intersections:
                        intersections = sorted(intersections, key=lambda pt: pt[0])
                        used_fixed_left[idx] = intersections[0]
                        used_fixed_right[idx] = intersections[-1]

    if outer_contour is not None:
        cv2.drawContours(frame_display, [outer_contour], -1, (0,255,255), 2)

    points = detect_black_blobs(frame)
    for pt in points:
        cv2.circle(frame_display, pt, 5, (0,255,0), -1)

    if len(points) == 10:
        sorted_by_x = sorted(points, key=lambda p: p[0])
        left_points = sorted_by_x[:5]
        right_points = sorted_by_x[5:]
        left_points = sorted(left_points, key=lambda p: p[1])
        right_points = sorted(right_points, key=lambda p: p[1])
        for i, pt in enumerate(left_points):
            cv2.putText(frame_display, f"L{i+1}", (pt[0]-20, pt[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
        for i, pt in enumerate(right_points):
            cv2.putText(frame_display, f"R{i+1}", (pt[0]+5, pt[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
        
        horizontal_lines = []
        for i in range(5):
            line_temp = (left_points[i], right_points[i])
            horizontal_lines.append(line_temp)
            cv2.line(frame_display, left_points[i], right_points[i], (255,0,0), 2)
        
        # 임시 변수 초기화
        current_additional_left = [None] * 5
        current_additional_right = [None] * 5
        current_command_left = [None] * 5
        current_command_right = [None] * 5
        error_left_arr = [0] * 5
        error_right_arr = [0] * 5

        updated = False  # 이번 프레임 업데이트 여부

        for idx, line_temp in enumerate(horizontal_lines):
            # 캘리브레이션 후엔 고정된 교점 사용, 아니면 임시 교점 사용
            if calibrated:
                left_int = fixed_intersections_left[idx]
                right_int = fixed_intersections_right[idx]
            else:
                left_int = used_fixed_left[idx]
                right_int = used_fixed_right[idx]
            if left_int is None or right_int is None:
                continue

            # 검정 점과 고정 교점 사이의 x 좌표 차이 및 각도 계산
            left_distance_px = np.sqrt((left_points[idx][0] - left_int[0])**2 + (left_points[idx][1] - left_int[1])**2)
            right_distance_px = np.sqrt((right_points[idx][0] - right_int[0])**2 + (right_points[idx][1] - right_int[1])**2)
            left_distance_cm = left_distance_px * (30.0 / frame_width)
            right_distance_cm = right_distance_px * (30.0 / frame_width)
            measured_left_angle = left_distance_cm * 60
            measured_right_angle = right_distance_cm * 60

            if not calibrated:
                left_angle_data[idx].append(measured_left_angle)
                right_angle_data[idx].append(measured_right_angle)
            else:
                # 먼저, 실시간으로 각도가 TARGET_ANGLE ± TOLERANCE 범위이면 바로 STOP
                if TARGET_ANGLE - TOLERANCE <= measured_left_angle <= TARGET_ANGLE + TOLERANCE:
                    current_command_left[idx] = "STOP"
                    error_left_arr[idx] = 0
                    updated = True
                # done 신호가 있으면 REV/FWD 업데이트
                elif motor_done_left[idx]:
                    current_additional_left[idx] = measured_left_angle
                    if measured_left_angle < TARGET_ANGLE - TOLERANCE:
                        current_command_left[idx] = "REV"
                        error_left_arr[idx] = TARGET_ANGLE - measured_left_angle
                    elif measured_left_angle > TARGET_ANGLE + TOLERANCE:
                        current_command_left[idx] = "FWD"
                        error_left_arr[idx] = measured_left_angle - TARGET_ANGLE
                    motor_done_left[idx] = False
                    updated = True

                if TARGET_ANGLE - TOLERANCE <= measured_right_angle <= TARGET_ANGLE + TOLERANCE:
                    current_command_right[idx] = "STOP"
                    error_right_arr[idx] = 0
                    updated = True
                elif motor_done_right[idx]:
                    current_additional_right[idx] = measured_right_angle
                    if measured_right_angle < TARGET_ANGLE - TOLERANCE:
                        current_command_right[idx] = "REV"
                        error_right_arr[idx] = TARGET_ANGLE - measured_right_angle
                    elif measured_right_angle > TARGET_ANGLE + TOLERANCE:
                        current_command_right[idx] = "FWD"
                        error_right_arr[idx] = measured_right_angle - TARGET_ANGLE
                    motor_done_right[idx] = False
                    updated = True

            # 화면에 고정 교점 표시
            cv2.circle(frame_display, tuple(left_int), 5, (0,0,255), -1)
            cv2.circle(frame_display, tuple(right_int), 5, (0,0,255), -1)

        if updated:
            print("Motor Commands:")
            for i in range(5):
                cmd = current_command_left[i] if current_command_left[i] is not None else "N/A"
                print(f"Motor L{i+1}: {cmd} (Error: {error_left_arr[i]:.2f}°)")
            for i in range(5):
                cmd = current_command_right[i] if current_command_right[i] is not None else "N/A"
                print(f"Motor R{i+1}: {cmd} (Error: {error_right_arr[i]:.2f}°)")

        for idx, line_temp in enumerate(horizontal_lines):
            if calibrated:
                left_int = fixed_intersections_left[idx]
                right_int = fixed_intersections_right[idx]
            else:
                left_int = used_fixed_left[idx]
                right_int = used_fixed_right[idx]
            if left_int is None or right_int is None:
                continue
            if current_command_left[idx] is not None:
                cv2.putText(frame_display, f"{current_command_left[idx]} {measured_left_angle:.1f}",
                            (left_int[0]-20, left_int[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
            if current_command_right[idx] is not None:
                cv2.putText(frame_display, f"{current_command_right[idx]} {measured_right_angle:.1f}",
                            (right_int[0]+10, right_int[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

    # 캘리브레이션 종료
    if not calibrated and (time.time() - calibration_start_time >= CALIBRATION_DURATION):
        calibrated = True
        for i in range(5):
            if left_angle_data[i]:
                calibration_left_baseline[i] = trimmed_mean(left_angle_data[i], trim_fraction=0.1)
            if right_angle_data[i]:
                calibration_right_baseline[i] = trimmed_mean(right_angle_data[i], trim_fraction=0.1)
        calibration_left_baseline = compute_relative(calibration_left_baseline)
        calibration_right_baseline = compute_relative(calibration_right_baseline)
        print("Calibration complete.")
        for i in range(5):
            print(f"Line {i+1}: Calibration Left = {calibration_left_baseline[i]:.2f}°, Calibration Right = {calibration_right_baseline[i]:.2f}°")
        
        # 고정된 교점 좌표 계산: 캘리브레이션 동안 저장된 좌표들의 단순 평균
        for i in range(5):
            if left_intersections_calib[i]:
                pts = np.array(left_intersections_calib[i])
                fixed_intersections_left[i] = tuple(np.mean(pts, axis=0).astype(np.int32))
            else:
                fixed_intersections_left[i] = None
            if right_intersections_calib[i]:
                pts = np.array(right_intersections_calib[i])
                fixed_intersections_right[i] = tuple(np.mean(pts, axis=0).astype(np.int32))
            else:
                fixed_intersections_right[i] = None
        print("Fixed intersection points determined by averaging calibration intersections.")
        
        time.sleep(2)
    
    cv2.imshow("Frame", frame_display)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    # 키보드 입력: "0" 입력 시 전역 done 신호 처리 (모든 모터)
    if key == ord('0'):
        for i in range(5):
            motor_done_left[i] = True
            motor_done_right[i] = True
        print("Simulated global done signal for ALL motors via keyboard")

cap.release()
cv2.destroyAllWindows()
