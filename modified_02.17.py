import cv2
import numpy as np
import time

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
    margin = int(0.1 * w)  # 좌우 10% 마진
    cropped = image[:, margin:w-margin]
    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=lambda c: cv2.contourArea(c))
        outer_contour = cv2.convexHull(largest)
        # cropped 영역이므로, x 좌표에 margin만큼 더해 원본 좌표로 복원
        outer_contour += np.array([[[margin, 0]]])
        return outer_contour
    return None

def line_intersection(line1, line2):
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0:
        return None  # 평행인 경우
    intersect_x = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / denom
    intersect_y = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / denom
    return (int(intersect_x), int(intersect_y))

def is_point_on_segment(pt, segment, tol=1e-6):
    (x, y) = pt
    (x1, y1), (x2, y2) = segment
    return (min(x1, x2) - tol <= x <= max(x1, x2) + tol) and \
           (min(y1, y2) - tol <= y <= max(y1, y2) + tol)

def get_intersections_for_line(line, contour):
    intersections = []
    pts = contour[:, 0, :]  # contour의 각 좌표 (N x 2)
    for i in range(len(pts)):
        seg = (tuple(pts[i]), tuple(pts[(i + 1) % len(pts)]))
        inter_pt = line_intersection(line, seg)
        if inter_pt is not None:
            if is_point_on_segment(inter_pt, line) and is_point_on_segment(inter_pt, seg):
                intersections.append(inter_pt)
    return intersections

def trimmed_mean(data, trim_fraction=0.1):
    """
    data 리스트에서 상하단 trim_fraction 만큼의 값을 제거한 후 평균을 계산합니다.
    """
    n = len(data)
    if n == 0:
        return 0
    sorted_data = sorted(data)
    trim_count = int(n * trim_fraction)
    if n - 2 * trim_count <= 0:
        return sum(sorted_data) / n
    trimmed_data = sorted_data[trim_count:n - trim_count]
    return sum(trimmed_data) / len(trimmed_data)

def compute_relative_angles(angles):
    """
    각 그룹의 각도 리스트에서 0이 아닌 값들 중 최소값을 기준으로 상대 회전각을 계산합니다.
    0은 결측값으로 간주되어 기준에 포함되지 않습니다.
    """
    valid_angles = [a for a in angles if a > 0]
    if not valid_angles:
        return angles
    baseline = min(valid_angles)
    return [a - baseline if a > 0 else 0 for a in angles]

def format_motor_commands(relative_left_angles, relative_right_angles):
    """
    모터 제어에 전달하기 좋은 형식의 문자열로 변환합니다.
    예: "L1:xx,L2:xx,...,R1:xx,R2:xx,..."
    """
    commands = []
    for i, angle in enumerate(relative_left_angles):
        commands.append(f"L{i+1}:{angle:.2f}")
    for i, angle in enumerate(relative_right_angles):
        commands.append(f"R{i+1}:{angle:.2f}")
    return ",".join(commands)

# 캘리브레이션 관련 변수
CALIBRATION_DURATION = 5.0  # 5초 동안 캘리브레이션
calibration_start_time = time.time()
calibrated = False
printed_final = False  # 최종 결과를 한 번만 출력하기 위한 플래그

# 각 라인(좌측 5개, 우측 5개)에 대해 회전각 데이터를 저장 (총 5개의 라인)
left_angle_data = [[] for _ in range(5)]
right_angle_data = [[] for _ in range(5)]
final_left_angles = [0] * 5
final_right_angles = [0] * 5

cap = cv2.VideoCapture(1)
while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 영상을 가져올 수 없습니다.")
        break

    frame_display = frame.copy()
    frame_width = frame.shape[1]

    # 물체 외곽선 검출 (좌우 마진 제외)
    outer_contour = detect_outer_object_contour(frame)
    if outer_contour is not None:
        cv2.drawContours(frame_display, [outer_contour], -1, (0, 255, 255), 2)
    
    # 검정 블롭(점) 검출 및 표시
    points = detect_black_blobs(frame)
    for pt in points:
        cv2.circle(frame_display, pt, 5, (0, 255, 0), -1)
    
    # 총 10개의 점이 검출된 경우, 좌측 5개와 우측 5개로 그룹화
    if len(points) == 10:
        sorted_by_x = sorted(points, key=lambda p: p[0])
        left_points = sorted_by_x[:5]
        right_points = sorted_by_x[5:]
        left_points = sorted(left_points, key=lambda p: p[1])
        right_points = sorted(right_points, key=lambda p: p[1])
        for i, pt in enumerate(left_points):
            cv2.putText(frame_display, f"L{i+1}", (pt[0]-20, pt[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        for i, pt in enumerate(right_points):
            cv2.putText(frame_display, f"R{i+1}", (pt[0]+5, pt[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        horizontal_lines = []
        for i in range(5):
            line = (left_points[i], right_points[i])
            horizontal_lines.append(line)
            cv2.line(frame_display, left_points[i], right_points[i], (255, 0, 0), 2)
        
        # 각 가로선과 외곽선 사이의 교점을 통해 회전각 계산
        if outer_contour is not None:
            for idx, line in enumerate(horizontal_lines):
                intersections = get_intersections_for_line(line, outer_contour)
                if intersections:
                    intersections = sorted(intersections, key=lambda pt: pt[0])
                    left_int = intersections[0]
                    right_int = intersections[-1]
                    
                    left_distance_px = left_int[0]           # 좌측 끝부터 왼쪽 교점까지
                    right_distance_px = frame_width - right_int[0]  # 우측 끝부터 오른쪽 교점까지
                    left_distance_cm = left_distance_px * (30.0 / frame_width)
                    right_distance_cm = right_distance_px * (30.0 / frame_width)
                    
                    left_angle = left_distance_cm * 60   # 1cm당 60° 회전
                    right_angle = right_distance_cm * 60
                    
                    if not calibrated:
                        left_angle_data[idx].append(left_angle)
                        right_angle_data[idx].append(right_angle)
                    else:
                        left_angle = final_left_angles[idx]
                        right_angle = final_right_angles[idx]
                    
                    for pt in intersections:
                        cv2.circle(frame_display, pt, 5, (0, 0, 255), -1)
    
    # 캘리브레이션 완료 체크 (5초 경과 시)
    if not calibrated and (time.time() - calibration_start_time >= CALIBRATION_DURATION):
        calibrated = True
        for i in range(5):
            if left_angle_data[i]:
                final_left_angles[i] = trimmed_mean(left_angle_data[i], trim_fraction=0.1)
            if right_angle_data[i]:
                final_right_angles[i] = trimmed_mean(right_angle_data[i], trim_fraction=0.1)
        # 상대 회전각 계산 (0은 결측값으로 처리)
        relative_left_angles = compute_relative_angles(final_left_angles)
        relative_right_angles = compute_relative_angles(final_right_angles)
        # 모터 제어에 전달하기 좋은 형식으로 명령 문자열 생성
        command_string = format_motor_commands(relative_left_angles, relative_right_angles)
        print("Calibration complete.")
        print("Motor Commands:", command_string)
        printed_final = True
    
    cv2.imshow("Frame", frame_display)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
