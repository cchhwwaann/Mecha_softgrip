import cv2
import numpy as np
import time
import serial

# Serial Setup
try:
    ser = serial.Serial("COM6", 115200, timeout=0.1)
    print("Ser_connected")
    time.sleep(2)
except Exception as e:
    print("Ser_failed")
    ser = None

# 아두이노에 "start" 명령 전송 (연결된 경우)
if ser is not None:
    ser.write("start\n".encode())
    ser.flush()  # 버퍼 플러시
    print("Sent start signal to Arduino.")

# 새로운 변수: Arduino로부터 "start" 신호를 받았는지 여부
startReceived = False

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
    margin_x = int(0.2 * w)
    margin_y = int(0.2 * h)
    cropped = image[margin_y: h - margin_y, margin_x: w - margin_x]
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
        # 보정: 크롭한 영역의 좌표를 원래 영상 좌표로 복원
        outer_contour += np.array([[[margin_x, margin_y]]])
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
    if n - 2 * trim_count <= 0:
        return sum(sorted_data) / n
    trimmed_data = sorted_data[trim_count:n-trim_count]
    return sum(trimmed_data) / len(trimmed_data)

# Calibration and control parameters
CALIBRATION_DURATION = 5.0  # 캘리브레이션 시간 (초)
# 캘리브레이션 시작 시간은 "start" 신호 받은 후에 초기화됨.
calibration_start_time = 0  
calibrated = False

TARGET_ANGLE = 120.0  # 목표 각도 (°)
TOLERANCE = 20.0      # 허용 오차 (°)

# 캘리브레이션 동안, 각 가로선과 외형 윤곽선의 교차점을 누적 (왼쪽, 오른쪽 따로)
calibration_intersections_left = [[] for _ in range(5)]
calibration_intersections_right = [[] for _ in range(5)]
# 캘리브레이션 완료 후, 각 가로선별 고정 교점 (왼쪽 5개, 오른쪽 5개 → 총 10개)
fixed_intersections_left = [None] * 5
fixed_intersections_right = [None] * 5

# CSV 명령 전송 딜레이
csv_delay = 2  # 초
last_csv_send_time = 0

done_received = False
first_measurement_done = False

cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 영상을 가져올 수 없습니다.")
        break

    frame_display = frame.copy()
    frame_height, frame_width = frame.shape[:2]

    # 시리얼 입력 처리: "start" 신호를 받으면 캘리브레이션 시작
    if ser is not None and ser.in_waiting:
        line = ser.readline().decode().strip()
        if not startReceived and line.lower() == "started":
            startReceived = True
            calibration_start_time = time.time()  # "start" 신호를 받은 시점으로 캘리브레이션 시작 시간 초기화
            print("Received start signal from Arduino. Starting calibration.")
        elif line.lower() == "done":
            done_received = True
            print("Received done signal from Arduino.")

    # 캘리브레이션은 startReceived가 되어야 진행됨.
    if not startReceived:
        cv2.putText(frame_display, "Waiting for start signal...", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Frame", frame_display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    # 외곽 윤곽선 검출 및 출력(파란선) (캘리브레이션 및 제어에 사용)
    outer_contour = detect_outer_object_contour(frame)
    if outer_contour is not None:
        cv2.drawContours(frame_display, [outer_contour], -1, (255, 0, 0), 2)

    # 5개의 가로선 계산 (항상 화면에 표시)
    horizontal_lines = []
    horizontal_y = []  # 각 가로선의 y 좌표
    for i in range(5):
        y_coord = int((i+1) * frame_height / 6)
        horizontal_lines.append(((0, y_coord), (frame_width, y_coord)))
        horizontal_y.append(y_coord)
        cv2.line(frame_display, (0, y_coord), (frame_width, y_coord), (0, 255, 0), 1)

    # 캘리브레이션 중이면, 각 가로선과 외곽 윤곽선의 교차점(왼쪽/오른쪽)을 누적
    if not calibrated and outer_contour is not None:
        for idx, line in enumerate(horizontal_lines):
            intersections = get_intersections_for_line(line, outer_contour)
            if intersections:
                intersections = sorted(intersections, key=lambda pt: pt[0])
                # 왼쪽 교차점과 오른쪽 교차점을 각각 누적
                calibration_intersections_left[idx].append(intersections[0])
                calibration_intersections_right[idx].append(intersections[-1])

    # 캘리브레이션 완료 후, 누적된 좌표들로 고정 교점 계산 (trimmed_mean 적용)
    if not calibrated and (time.time() - calibration_start_time >= CALIBRATION_DURATION):
        for idx in range(5):
            if calibration_intersections_left[idx]:
                x_vals = [pt[0] for pt in calibration_intersections_left[idx]]
                y_vals = [pt[1] for pt in calibration_intersections_left[idx]]
                mean_x = int(trimmed_mean(x_vals, trim_fraction=0.1))
                mean_y = int(trimmed_mean(y_vals, trim_fraction=0.1))
                fixed_intersections_left[idx] = (mean_x, mean_y)
            else:
                fixed_intersections_left[idx] = None
            if calibration_intersections_right[idx]:
                x_vals = [pt[0] for pt in calibration_intersections_right[idx]]
                y_vals = [pt[1] for pt in calibration_intersections_right[idx]]
                mean_x = int(trimmed_mean(x_vals, trim_fraction=0.1))
                mean_y = int(trimmed_mean(y_vals, trim_fraction=0.1))
                fixed_intersections_right[idx] = (mean_x, mean_y)
            else:
                fixed_intersections_right[idx] = None
        calibrated = True
        first_measurement_done = False
        time.sleep(0.1)

    # 캘리브레이션 완료 후, 고정 교점(왼쪽/오른쪽 모두)을 노란색 원으로 표시
    if calibrated:
        for pt in fixed_intersections_left:
            if pt is not None:
                cv2.circle(frame_display, pt, 5, (0, 255, 255), -1)
        for pt in fixed_intersections_right:
            if pt is not None:
                cv2.circle(frame_display, pt, 5, (0, 255, 255), -1)

    # 측정 및 제어: 검은 블롭과 각 고정 교점 사이의 x좌표 차이를 이용해 제어 각도 산출
    if calibrated:
        blobs = detect_black_blobs(frame)
        for blob in blobs:
            cv2.circle(frame_display, blob, 5, (0, 0, 255), -1)

        # 각 가로선별로 검은 블롭을 좌/우로 따로 연관시킴
        associated_blobs_left = [None] * 5
        associated_blobs_right = [None] * 5
        for i, y_coord in enumerate(horizontal_y):
            left_candidates = []
            right_candidates = []
            for blob in blobs:
                if abs(blob[1] - y_coord) < 20:  # y좌표 차이가 20픽셀 이내인 경우
                    if fixed_intersections_left[i] is not None:
                        if blob[0] <= fixed_intersections_left[i][0]:
                            left_candidates.append(blob)
                    else:
                        left_candidates.append(blob)
                    if fixed_intersections_right[i] is not None:
                        if blob[0] >= fixed_intersections_right[i][0]:
                            right_candidates.append(blob)
                    else:
                        right_candidates.append(blob)
            if left_candidates:
                associated_blobs_left[i] = min(left_candidates, key=lambda b: abs(b[1]-y_coord))
            else:
                associated_blobs_left[i] = min(blobs, key=lambda b: abs(b[1]-y_coord)) if blobs else None
            if right_candidates:
                associated_blobs_right[i] = min(right_candidates, key=lambda b: abs(b[1]-y_coord))
            else:
                associated_blobs_right[i] = min(blobs, key=lambda b: abs(b[1]-y_coord)) if blobs else None

        current_time = time.time()
        if (not first_measurement_done or done_received) and (current_time - last_csv_send_time >= csv_delay):
            print("CSV Command Output:")
            # 왼쪽 고정 교점에 대한 제어
            for idx in range(5):
                fixed_pt = fixed_intersections_left[idx]
                blob_pt = associated_blobs_left[idx]
                if fixed_pt is None or blob_pt is None:
                    # 교점이 없으면 해당 모터는 STOP 명령 전송
                    cmd_left = "STOP"
                    error_left = 0
                else:
                    x_diff_px = abs(blob_pt[0] - fixed_pt[0])
                    control_angle = int(x_diff_px * (30.0 / frame_width) * 60)
                    if TARGET_ANGLE - TOLERANCE <= control_angle <= TARGET_ANGLE + TOLERANCE:
                        cmd_left = "STOP"
                        error_left = 0
                    elif control_angle < TARGET_ANGLE - TOLERANCE:
                        cmd_left = "REV"
                        error_left = int(TARGET_ANGLE - control_angle)
                    else:
                        cmd_left = "FWD"
                        error_left = int(control_angle - TARGET_ANGLE)
                csv_command = f"L{idx+1},{cmd_left},{error_left}"
                print(csv_command)
                if ser is not None:
                    ser.write((csv_command + "\n").encode())
            # 오른쪽 고정 교점에 대한 제어
            for idx in range(5):
                fixed_pt = fixed_intersections_right[idx]
                blob_pt = associated_blobs_right[idx]
                if fixed_pt is None or blob_pt is None:
                    cmd_right = "STOP"
                    error_right = 0
                else:
                    x_diff_px = abs(blob_pt[0] - fixed_pt[0])
                    control_angle = int(x_diff_px * (30.0 / frame_width) * 60)
                    if TARGET_ANGLE - TOLERANCE <= control_angle <= TARGET_ANGLE + TOLERANCE:
                        cmd_right = "STOP"
                        error_right = 0
                    elif control_angle < TARGET_ANGLE - TOLERANCE:
                        cmd_right = "REV"
                        error_right = int(TARGET_ANGLE - control_angle)
                    else:
                        cmd_right = "FWD"
                        error_right = int(control_angle - TARGET_ANGLE)
                csv_command = f"R{idx+1},{cmd_right},{error_right}"
                print(csv_command)
                if ser is not None:
                    ser.write((csv_command + "\n").encode())

        
            
            first_measurement_done = True
            done_received = False
            last_csv_send_time = current_time

        # 화면에 각 고정 교점 근처에 제어 각도 표시 (왼쪽/오른쪽 모두)
        for idx in range(5):
            fixed_pt = fixed_intersections_left[idx]
            blob_pt = associated_blobs_left[idx]
            if fixed_pt is not None and blob_pt is not None:
                x_diff_px = abs(blob_pt[0] - fixed_pt[0])
                control_angle = int(x_diff_px * (23.0 / frame_width) * 60)
                cv2.putText(frame_display, f"{control_angle}°", (fixed_pt[0]+10, fixed_pt[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        for idx in range(5):
            fixed_pt = fixed_intersections_right[idx]
            blob_pt = associated_blobs_right[idx]
            if fixed_pt is not None and blob_pt is not None:
                x_diff_px = abs(blob_pt[0] - fixed_pt[0])
                control_angle = int(x_diff_px * (23.0 / frame_width) * 60)
                cv2.putText(frame_display, f"{control_angle}°", (fixed_pt[0]-40, fixed_pt[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    cv2.imshow("Frame", frame_display)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        if ser is not None:
            ser.write("finish\n".encode())
        break

cap.release()
cv2.destroyAllWindows()
