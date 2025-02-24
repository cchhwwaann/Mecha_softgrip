import cv2
import numpy as np
import time
import serial

# Serial Setup
try:
    ser = serial.Serial("COM3", 115200, timeout=0.1)
    print("Ser_connected")
except Exception as e:
    print("Ser_failed")
    ser = None

# ---------------------------
# Helper functions
# ---------------------------
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
        seg = (tuple(pts[i]), tuple(pts[(i+1) % len(pts)]))
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
        return sum(sorted_data) / n
    trimmed_data = sorted_data[trim_count:n-trim_count]
    return sum(trimmed_data) / len(trimmed_data)

def compute_relative(baseline_angles):
    valid = [a for a in baseline_angles if a > 0]
    if not valid:
        return baseline_angles
    base = min(valid)
    return [a - base if a > 0 else 0 for a in baseline_angles]

# ---------------------------
# 설정 및 변수
# ---------------------------
CALIBRATION_DURATION = 5.0  # 캘리브레이션 시간 (초)
calibration_start_time = time.time()
calibrated = False

left_angle_data = [[] for _ in range(5)]
right_angle_data = [[] for _ in range(5)]
calibration_left_baseline = [0] * 5  
calibration_right_baseline = [0] * 5

TARGET_ANGLE = 60.0
TOLERANCE = 10.0

# 캘리브레이션 동안 교점 좌표 저장 (각각 5개)
left_intersections_calib = [[] for _ in range(5)]
right_intersections_calib = [[] for _ in range(5)]
# 캘리브레이션 후 고정된 교점 좌표 (각각 5개)
fixed_intersections_left = [None] * 5
fixed_intersections_right = [None] * 5

# 아두이노 done 신호 플래그 및 최초 측정 여부
done_received = False
first_measurement_done = False

# CSV 명령 전송 딜레이 (테스트 단계)
csv_delay = 2  # 초
last_csv_send_time = 0

# ---------------------------
# 메인 루프
# ---------------------------
cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 영상을 가져올 수 없습니다.")
        break

    frame_display = frame.copy()
    frame_width = frame.shape[1]

    # 아두이노로부터 done 신호 수신
    if ser is not None and ser.in_waiting:
        line = ser.readline().decode().strip()
        if line.lower() == "done":
            done_received = True
            print("Received done signal from Arduino.")

    # 외형 윤곽선 검출
    outer_contour = detect_outer_object_contour(frame)

    # 캘리브레이션 중이면 교점 좌표 저장
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
                    left_intersections_calib[idx].append(intersections[0])
                    right_intersections_calib[idx].append(intersections[-1])

    # 캘리브레이션 완료 및 고정 교점 계산 (5초 후)
    if not calibrated and (time.time() - calibration_start_time >= CALIBRATION_DURATION):
        calibrated = True
        # 내부 처리용으로 캘리브레이션 동안 수집된 각도 데이터의 평균 계산
        for i in range(5):
            if left_angle_data[i]:
                calibration_left_baseline[i] = trimmed_mean(left_angle_data[i], trim_fraction=0.1)
            if right_angle_data[i]:
                calibration_right_baseline[i] = trimmed_mean(right_angle_data[i], trim_fraction=0.1)
        calibration_left_baseline = compute_relative(calibration_left_baseline)
        calibration_right_baseline = compute_relative(calibration_right_baseline)
        # 캘리브레이션 관련 출력은 제거됨.
        # 고정된 교점 계산: 누적된 교점의 평균값
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
        first_measurement_done = False
        # 짧은 대기 후 캘리브레이션 완료 상태로 진입 (이 대기는 여기서만 사용하며, 이후 딜레이는 비차단 방식으로 처리)
        time.sleep(0.1)

    # 영상에 검출 결과 표시 (교점, 블롭, 라인 등)
    if outer_contour is not None:
        cv2.drawContours(frame_display, [outer_contour], -1, (0, 255, 255), 2)
    points = detect_black_blobs(frame)
    for pt in points:
        cv2.circle(frame_display, pt, 5, (0, 255, 0), -1)

    if len(points) == 10:
        sorted_by_x = sorted(points, key=lambda p: p[0])
        left_points = sorted_by_x[:5]
        right_points = sorted_by_x[5:]
        left_points = sorted(left_points, key=lambda p: p[1])
        right_points = sorted(right_points, key=lambda p: p[1])
        # 좌측/우측 블롭 라벨 표시
        for i, pt in enumerate(left_points):
            cv2.putText(frame_display, f"L{i+1}", (pt[0]-20, pt[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        for i, pt in enumerate(right_points):
            cv2.putText(frame_display, f"R{i+1}", (pt[0]+5, pt[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        # 각 라인의 수평선 그리기
        horizontal_lines = []
        for i in range(5):
            line_temp = (left_points[i], right_points[i])
            horizontal_lines.append(line_temp)
            cv2.line(frame_display, left_points[i], right_points[i], (255, 0, 0), 2)
        
        # 캘리브레이션 완료 후 최초 측정 또는 "done" 신호가 있으면, 딜레이를 타임스탬프로 구현하여 CSV 명령 전송
        current_time = time.time()
        if calibrated and (not first_measurement_done or done_received) and (current_time - last_csv_send_time >= csv_delay):
            print("CSV 명령 출력:")
            for idx, line_temp in enumerate(horizontal_lines):
                left_int = fixed_intersections_left[idx]
                right_int = fixed_intersections_right[idx]
                if left_int is None or right_int is None:
                    continue
                left_distance_px = np.sqrt((left_points[idx][0] - left_int[0])**2 + (left_points[idx][1] - left_int[1])**2)
                right_distance_px = np.sqrt((right_points[idx][0] - right_int[0])**2 + (right_points[idx][1] - right_int[1])**2)
                left_distance_cm = left_distance_px * (30.0 / frame_width)
                right_distance_cm = right_distance_px * (30.0 / frame_width)
                measured_left_angle = int(left_distance_cm * 60)
                measured_right_angle = int(right_distance_cm * 60)

                if TARGET_ANGLE - TOLERANCE <= measured_left_angle <= TARGET_ANGLE + TOLERANCE:
                    cmd_left = "STOP"
                    error_left = 0
                elif measured_left_angle < TARGET_ANGLE - TOLERANCE:
                    cmd_left = "REV"
                    error_left = int(TARGET_ANGLE - measured_left_angle)
                else:
                    cmd_left = "FWD"
                    error_left = int(measured_left_angle - TARGET_ANGLE)

                if TARGET_ANGLE - TOLERANCE <= measured_right_angle <= TARGET_ANGLE + TOLERANCE:
                    cmd_right = "STOP"
                    error_right = 0
                elif measured_right_angle < TARGET_ANGLE - TOLERANCE:
                    cmd_right = "REV"
                    error_right = int(TARGET_ANGLE - measured_right_angle)
                else:
                    cmd_right = "FWD"
                    error_right = int(measured_right_angle - TARGET_ANGLE)

                csv_left = f"L{idx+1},{cmd_left},{error_left}"
                csv_right = f"R{idx+1},{cmd_right},{error_right}"
                print(csv_left)
                print(csv_right)
                if ser is not None:
                    ser.write((csv_left + "\n").encode())
                    ser.write((csv_right + "\n").encode())
            first_measurement_done = True
            done_received = False
            last_csv_send_time = current_time  # 딜레이 시작 시각 업데이트

        # 화면에 측정된 각도 값 정수로 표시
        for idx, line_temp in enumerate(horizontal_lines):
            left_int = fixed_intersections_left[idx]
            right_int = fixed_intersections_right[idx]
            if left_int is None or right_int is None:
                continue
            left_distance_px = np.sqrt((left_points[idx][0] - left_int[0])**2 + (left_points[idx][1] - left_int[1])**2)
            right_distance_px = np.sqrt((right_points[idx][0] - right_int[0])**2 + (right_points[idx][1] - right_int[1])**2)
            left_distance_cm = left_distance_px * (30.0 / frame_width)
            right_distance_cm = right_distance_px * (30.0 / frame_width)
            measured_left_angle = int(left_distance_cm * 60)
            measured_right_angle = int(right_distance_cm * 60)
            cv2.putText(frame_display, f"{measured_left_angle}°",
                        (left_int[0]-20, left_int[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.putText(frame_display, f"{measured_right_angle}°",
                        (right_int[0]+10, right_int[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # 캘리브레이션 완료 후 고정 교점을 항상 프레임에 표시
    if calibrated:
        for idx in range(5):
            if fixed_intersections_left[idx] is not None:
                cv2.circle(frame_display, fixed_intersections_left[idx], 5, (0, 255, 255), -1)
            if fixed_intersections_right[idx] is not None:
                cv2.circle(frame_display, fixed_intersections_right[idx], 5, (0, 255, 255), -1)

    cv2.imshow("Frame", frame_display)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        if ser is not None:
            ser.write("finish\n".encode())
        break

cap.release()
cv2.destroyAllWindows()
