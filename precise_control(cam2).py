import cv2
import numpy as np

# 카메라 캡처 객체 생성
cap = cv2.VideoCapture(0)

# 실제 거리 변환에 필요한 변수 (단위: cm)
object_distance_cm = 30.0       # 카메라와 물체 사이의 실제 거리 (cm)
focal_length_pixels = 600.0       # 초점거리 (픽셀 단위; 실제 환경에서는 카메라 보정 필요)
conversion_factor = object_distance_cm / focal_length_pixels  # 픽셀 -> cm 변환 계수 (여기서는 0.05)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 필요에 따라 좌우 반전
    frame = cv2.flip(frame, 1)
    h, w = frame.shape[:2]
    frame_center = (w // 2, h // 2)

    ### 1. 빨간 점(집게 마킹) 검출 ###
    # HSV 색공간 변환 후 빨간색 범위(두 범위)를 마스크로 검출
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # 노이즈 제거: 열림/닫힘 연산
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    # 빨간 영역의 컨투어 검출
    contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_centers = []  # 검출된 빨간 점의 중심 좌표 저장

    for cnt in contours_red:
        area = cv2.contourArea(cnt)
        if area > 50:  # 노이즈 필터링 (필요시 값 조정)
            # 최소 외접원을 이용하여 중심 좌표 계산
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            red_centers.append(center)
            # 시각화: 노란 테두리와 빨간 중심점
            cv2.circle(frame, center, int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

    # 좌측(화면 왼쪽)과 우측(화면 오른쪽) 빨간 점 그룹 분류
    left_points = []
    right_points = []
    for pt in red_centers:
        if pt[0] < w // 2:
            left_points.append(pt)
        else:
            right_points.append(pt)

    # 집게의 내측(물체 쪽) 빨간 점 선택  
    # - 좌측 집게: 오른쪽에 있는 (x값이 큰) 점  
    # - 우측 집게: 왼쪽에 있는 (x값이 작은) 점
    left_inner = max(left_points, key=lambda p: p[0]) if left_points else None
    right_inner = min(right_points, key=lambda p: p[0]) if right_points else None

    if left_inner is not None:
        cv2.circle(frame, left_inner, 8, (255, 0, 0), -1)
    if right_inner is not None:
        cv2.circle(frame, right_inner, 8, (255, 0, 0), -1)

    ### 2. 화면 중앙부 물체의 외형선(컨투어) 검출 ###
    # 그레이스케일 변환 → 가우시안 블러 → Canny 에지 검출
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    contours_all, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 중앙에 위치하거나 (또는 면적이 큰) 컨투어를 물체로 판단
    obj_contour = None
    min_center_dist = 1e9
    for cnt in contours_all:
        area = cv2.contourArea(cnt)
        if area < 500:
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        dist = np.hypot(cx - frame_center[0], cy - frame_center[1])
        if dist < min_center_dist:
            min_center_dist = dist
            obj_contour = cnt

    if obj_contour is not None:
        # 중앙 물체의 외형선을 녹색으로 표시
        cv2.drawContours(frame, [obj_contour], -1, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "중앙 물체 미검출", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    ### 3. 빨간 점과 중앙 물체 외형선 사이의 거리(픽셀)를 cm로 환산 ###
    if obj_contour is not None:
        for pt in red_centers:
            # cv2.pointPolygonTest: 컨투어와 점 사이의 최소 거리 (픽셀 단위) 반환
            pixel_distance = cv2.pointPolygonTest(obj_contour, pt, True)
            pixel_distance = abs(pixel_distance)  # 내부이면 음수로 반환하므로 절대값 사용
            real_distance_cm = pixel_distance * conversion_factor  # cm 단위로 변환
            cv2.putText(frame, f"{real_distance_cm:.2f}cm", (pt[0]+10, pt[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    ### 4. 좌측과 우측 집게 내측 빨간 점 사이의 gap 계산 (픽셀→cm) ###
    if (left_inner is not None) and (right_inner is not None):
        gap_pixels = np.hypot(right_inner[0] - left_inner[0], right_inner[1] - left_inner[1])
        gap_cm = gap_pixels * conversion_factor  # cm로 변환
        cv2.line(frame, left_inner, right_inner, (255, 0, 255), 2)
        cv2.putText(frame, f"Gap: {gap_cm:.2f}cm", (frame_center[0]-50, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
    else:
        cv2.putText(frame, "집게 빨간 점 부족", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    ### 5. 결과 영상 출력 ###
    cv2.imshow("Frame", frame)
    # 디버깅을 위해 필요 시 아래 창들을 활성화할 수 있음.
    # cv2.imshow("Red Mask", red_mask)
    # cv2.imshow("Edges", edges)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
