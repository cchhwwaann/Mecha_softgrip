import cv2
import numpy as np

def detect_black_blobs(image):
    """
    SimpleBlobDetector를 사용해 검정색(어두운) 점들을 검출합니다.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 0  # 검정색
    params.filterByArea = True
    params.minArea = 50     # 필요에 따라 조정
    params.maxArea = 5000   # 필요에 따라 조정
    params.filterByCircularity = True
    params.minCircularity = 0.7
    params.filterByConvexity = False
    params.filterByInertia = False
    
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(gray)
    points = [(int(kp.pt[0]), int(kp.pt[1])) for kp in keypoints]
    return points

def detect_outer_object_contour(image):
    """
    전체 프레임에서 좌우 마진(전체 너비의 10%)을 제외한 영역에서
    그레이스케일, 가우시안 블러, adaptiveThreshold, 모폴로지 연산을 적용해
    가장 큰 컨투어의 외곽(Convex Hull)만 추출합니다.
    검출된 외곽선의 좌표는 원본 프레임 기준으로 복원됩니다.
    """
    h, w = image.shape[:2]
    margin = int(0.1 * w)  # 좌우 10% 마진 (필요에 따라 조정)
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
    """
    두 선분(line1, line2) 사이의 교점을 계산합니다.
    각 선분은 ((x1, y1), (x2, y2)) 형태의 튜플입니다.
    평행하거나 교점이 없으면 None을 반환합니다.
    """
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0:
        return None  # 평행한 경우

    intersect_x = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / denom
    intersect_y = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / denom
    return (int(intersect_x), int(intersect_y))

def is_point_on_segment(pt, segment, tol=1e-6):
    """
    pt가 segment 선분의 양 끝점을 포함한 범위 내에 있는지 확인합니다.
    """
    (x, y) = pt
    (x1, y1), (x2, y2) = segment
    return (min(x1, x2) - tol <= x <= max(x1, x2) + tol) and \
           (min(y1, y2) - tol <= y <= max(y1, y2) + tol)

def get_intersections_for_line(line, contour):
    """
    주어진 선(line)과 물체 외곽선(Convex Hull)의 각 선분 사이의 교점을 계산합니다.
    """
    intersections = []
    pts = contour[:, 0, :]  # 컨투어의 각 좌표 (N x 2)
    for i in range(len(pts)):
        seg = (tuple(pts[i]), tuple(pts[(i + 1) % len(pts)]))
        inter_pt = line_intersection(line, seg)
        if inter_pt is not None:
            if is_point_on_segment(inter_pt, line) and is_point_on_segment(inter_pt, seg):
                intersections.append(inter_pt)
    return intersections

# --- 메인 코드 ---
cap = cv2.VideoCapture(1)  # 카메라 인덱스에 맞게 조정

while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 영상을 가져올 수 없습니다.")
        break

    frame_display = frame.copy()
    frame_width = frame.shape[1]

    # 1. 전체 프레임에서 좌우 마진을 제외한 영역에서 물체 외형의 외곽(Convex Hull) 검출
    outer_contour = detect_outer_object_contour(frame)
    if outer_contour is not None:
        cv2.drawContours(frame_display, [outer_contour], -1, (0, 255, 255), 2)
    
    # 2. 검정 블롭(점) 검출 (개별 번호는 표시하지 않음)
    points = detect_black_blobs(frame)
    for pt in points:
        cv2.circle(frame_display, pt, 5, (0, 255, 0), -1)

    # 3. 총 10개의 점이 검출되었을 때, 좌측 5개와 우측 5개로 그룹핑 후 "L", "R" 라벨만 표시
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
        
        # 4. 각 가로선과 물체 외곽선(Convex Hull) 사이의 교점 계산 및 표시
        #    교점에는 라벨 없이 빨간 점만 표시하고, 좌우 끝과의 거리를 콘솔에 출력
        if outer_contour is not None:
            for idx, line in enumerate(horizontal_lines):
                intersections = get_intersections_for_line(line, outer_contour)
                # 교점이 여러 개일 경우 x 좌표 기준으로 정렬 후, 가장 왼쪽, 가장 오른쪽 선택
                if intersections:
                    intersections = sorted(intersections, key=lambda pt: pt[0])
                    left_int = intersections[0]
                    right_int = intersections[-1]
                    left_distance = left_int[0]  # 좌측 끝(0)에서 교점까지의 거리
                    right_distance = frame_width - right_int[0]  # 우측 끝(frame_width)에서 교점까지의 거리
                    print(f"Horizontal line {idx+1}: Left distance = {left_distance}, Right distance = {right_distance}")
                
                # 교점은 라벨 없이 빨간 원으로 표시
                for pt in intersections:
                    cv2.circle(frame_display, pt, 5, (0, 0, 255), -1)
    
    cv2.imshow("Frame", frame_display)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
