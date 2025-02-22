import cv2

for i in range(5):  # 캠 0~4까지 시도해보기
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"카메라 {i}번 인식됨")
        cap.release()
    else:
        print(f"카메라 {i}번 없음")
