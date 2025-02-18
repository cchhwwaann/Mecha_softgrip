import cv2
import numpy as np

# 웹캠 실행
cap = cv2.VideoCapture(0)  # 0번 카메라 (기본 웹캠)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 이미지 전처리
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 그레이스케일 변환
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)  # 가우시안 블러 적용

    # Otsu 이진화 적용
    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 모폴로지 연산 (잡음 제거 & 구멍 채우기)
    kernel = np.ones((5, 5), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    # 컨투어 검출
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 컨투어 필터링 (작은 노이즈 제거)
    min_area = 500  # 최소 크기 기준
    filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

    # 컨투어 근사화 및 외형선 그리기
    for contour in filtered_contours:
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)  # 초록색 컨투어

    # 화면 출력
    cv2.imshow("Contours", frame)

    # 종료 키 (ESC)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
