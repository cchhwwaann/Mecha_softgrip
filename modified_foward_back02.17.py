import cv2
import numpy as np
import time

# ======================
# Helper functions
# ======================

def detect_black_blobs(image):
    """입력 이미지에서 SimpleBlobDetector를 사용해 검정색 블롭(점)들의 중심 좌표를 반환"""
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
    """좌우 10% 마진 제외 영역에서 외곽선(Convex Hull)을 추출 후 원본 좌표로 복원"""
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
    """두 선분(line1, line2) 사이의 교점을 계산, 평행하면 None 반환"""
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2
    denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if denom == 0:
        return None
    intersect_x = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)) / denom
    intersect_y = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)) / denom
    return (int(intersect_x), int(intersect_y))

def is_point_on_segment(pt, segment, tol=1e-6):
    """pt가 선분(segment) 내에 있는지 확인"""
    (x, y) = pt
    (x1, y1), (x2, y2) = segment
    return (min(x1, x2)-tol <= x <= max(x1, x2)+tol) and (min(y1, y2)-tol <= y <= max(y1, y2)+tol)

def get_intersections_for_line(line, contour):
    """가로선(line)과 외곽선(contour) 사이의 모든 교점을 반환"""
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
    """data에서 상하단 trim_fraction 만큼 제거한 후 평균 계산"""
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
    """각 그룹의 회전각 리스트에서 0이 아닌 값 중 최소값을 기준으로 상대 회전각 계산"""
    valid = [a for a in baseline_angles if a > 0]
    if not valid:
        return baseline_angles
    base = min(valid)
    return [a - base if a > 0 else 0 for a in baseline_angles]

# ==============================
# 설정 및 변수 (체크포인트 1 & 2)
# ==============================
CALIBRATION_DURATION = 5.0  # 초
calibration_start_time = time.time()
calibrated = False

left_angle_data = [[] for _ in range(5)]
right_angle_data = [[] for _ in range(5)]
calibration_left_baseline = [0] * 5
calibration_right_baseline = [0] * 5

# 실시간 피드백 관련
left_motor_stopped = [False] * 5
right_motor_stopped = [False] * 5

left_prev_command = [None] * 5
right_prev_command = [None] * 5

TARGET_ANGLE = 100.0
TOLERANCE = 10.0  # ±10°

additional_commands_printed = False

# 변화율 판단을 위한 설정 (각도 단위)
STABLE_THRESHOLD = 10.0   # 변화량 ?° 이하이면 안정적
STABLE_COUNT_THRESHOLD = 10 # 연속 ? 프레임 이상 안정적이면 명령 완료로 간주

# 이전 프레임의 측정값과 안정 카운트 (각 모터별)
prev_measured_left = [None] * 5
prev_measured_right = [None] * 5
stable_count_left = [0] * 5
stable_count_right = [0] * 5

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

    outer_contour = detect_outer_object_contour(frame)
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
            line = (left_points[i], right_points[i])
            horizontal_lines.append(line)
            cv2.line(frame_display, left_points[i], right_points[i], (255,0,0), 2)
        
        # 실시간 추가 회전각 및 명령 결정
        current_additional_left = [None] * 5
        current_additional_right = [None] * 5
        current_command_left = [None] * 5
        current_command_right = [None] * 5
        error_left_arr = [0] * 5
        error_right_arr = [0] * 5

        for idx, line in enumerate(horizontal_lines):
            intersections = get_intersections_for_line(line, outer_contour)
            if intersections:
                intersections = sorted(intersections, key=lambda pt: pt[0])
                left_int = intersections[0]
                right_int = intersections[-1]

                # 측정: 검은 점과 교점 사이의 x 좌표 차이
                left_distance_px = abs(left_points[idx][0] - left_int[0])
                right_distance_px = abs(right_points[idx][0] - right_int[0])

                left_distance_cm = left_distance_px * (30.0 / frame_width)
                right_distance_cm = right_distance_px * (30.0 / frame_width)

                measured_left_angle = left_distance_cm * 60
                measured_right_angle = right_distance_cm * 60

                if not calibrated:
                    left_angle_data[idx].append(measured_left_angle)
                    right_angle_data[idx].append(measured_right_angle)
                else:
                    current_additional_left[idx] = measured_left_angle
                    current_additional_right[idx] = measured_right_angle

                    # 변화율 기반 안정 판단 (각도 변화량이 STABLE_THRESHOLD 미만이면 안정 카운트 증가)
                    if prev_measured_left[idx] is not None:
                        if abs(measured_left_angle - prev_measured_left[idx]) < STABLE_THRESHOLD:
                            stable_count_left[idx] += 1
                        else:
                            stable_count_left[idx] = 0
                    prev_measured_left[idx] = measured_left_angle

                    if prev_measured_right[idx] is not None:
                        if abs(measured_right_angle - prev_measured_right[idx]) < STABLE_THRESHOLD:
                            stable_count_right[idx] += 1
                        else:
                            stable_count_right[idx] = 0
                    prev_measured_right[idx] = measured_right_angle

                    # 여기서 안정 상태(즉, 명령 완료)가 되었다고 판단할 조건:
                    # 각 모터별로 연속 STABLE_COUNT_THRESHOLD 프레임 이상 변화가 1° 미만이면 "명령 완료"
                    # 단, 이 안정 여부는 추가 회전각 명령에 영향을 주지 않고, 실제 명령 결정은 TARGET_ANGLE과 비교함.
                    if measured_left_angle < TARGET_ANGLE - TOLERANCE:
                        current_command_left[idx] = "REV"
                        error_left_arr[idx] = TARGET_ANGLE - measured_left_angle
                    elif measured_left_angle > TARGET_ANGLE + TOLERANCE:
                        current_command_left[idx] = "FWD"
                        error_left_arr[idx] = measured_left_angle - TARGET_ANGLE
                    else:
                        current_command_left[idx] = "STOP"
                        error_left_arr[idx] = 0

                    if measured_right_angle < TARGET_ANGLE - TOLERANCE:
                        current_command_right[idx] = "REV"
                        error_right_arr[idx] = TARGET_ANGLE - measured_right_angle
                    elif measured_right_angle > TARGET_ANGLE + TOLERANCE:
                        current_command_right[idx] = "FWD"
                        error_right_arr[idx] = measured_right_angle - TARGET_ANGLE
                    else:
                        current_command_right[idx] = "STOP"
                        error_right_arr[idx] = 0

        # 첫 실시간 피드백 프레임에서, 안정 판단과 상관없이 10개 모터의 추가 회전각을 출력
        if calibrated and not additional_commands_printed:
            print("Initial Additional Rotation Commands (to contact object):")
            for i in range(5):
                if current_additional_left[i] is not None:
                    print(f"Motor L{i+1}: {current_additional_left[i]:.2f}°")
                else:
                    print(f"Motor L{i+1}: N/A")
            for i in range(5):
                if current_additional_right[i] is not None:
                    print(f"Motor R{i+1}: {current_additional_right[i]:.2f}°")
                else:
                    print(f"Motor R{i+1}: N/A")
            additional_commands_printed = True

        # 만약 명령 종류(REV, FWD, STOP)가 이전과 달라진 모터가 하나라도 있다면 전체 10개 모터 상태 출력
        command_changed = False
        for i in range(5):
            if current_command_left[i] != left_prev_command[i] or current_command_right[i] != right_prev_command[i]:
                command_changed = True
                break
        if command_changed:
            print("Motor Commands:")
            for i in range(5):
                cmd = current_command_left[i] if current_command_left[i] is not None else "N/A"
                print(f"Motor L{i+1}: {cmd} (Error: {error_left_arr[i]:.2f}°)")
            for i in range(5):
                cmd = current_command_right[i] if current_command_right[i] is not None else "N/A"
                print(f"Motor R{i+1}: {cmd} (Error: {error_right_arr[i]:.2f}°)")
            for i in range(5):
                left_prev_command[i] = current_command_left[i]
            for i in range(5):
                right_prev_command[i] = current_command_right[i]

        # 화면 출력: 각 모터별 명령 텍스트 표시 (이전과 동일)
        for idx, line in enumerate(horizontal_lines):
            intersections = get_intersections_for_line(line, outer_contour)
            if intersections:
                intersections = sorted(intersections, key=lambda pt: pt[0])
                left_int = intersections[0]
                right_int = intersections[-1]
                if current_command_left[idx] == "STOP":
                    cv2.putText(frame_display, "STOP", (left_int[0]-20, left_int[1]-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
                else:
                    cv2.putText(frame_display, f"{current_command_left[idx]} {measured_left_angle:.1f}", 
                                (left_int[0]-20, left_int[1]-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
                if current_command_right[idx] == "STOP":
                    cv2.putText(frame_display, "STOP", (right_int[0]+10, right_int[1]-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
                else:
                    cv2.putText(frame_display, f"{current_command_right[idx]} {measured_right_angle:.1f}",
                                (right_int[0]+10, right_int[1]-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
                for pt in intersections:
                    cv2.circle(frame_display, pt, 5, (0,0,255), -1)
    
    # 캘리브레이션 완료 체크 (5초 경과 시)
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
        time.sleep(2)
    
    cv2.imshow("Frame", frame_display)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# --> 첫 외형 픽스 해야함 ( 나중에 리버스 넣을때 외형선 이슈생길수도있으니까 나머지 그립부분이 붙으면 외형선 못 땀)
# -->모터가 다 움직였다 판단하는 시점은 done으로 받는다.(serial통신신)
# --> done신호받되, 실시간으로보는건 유지하면서 너무 과하게 움직이면 stop할수있도록록