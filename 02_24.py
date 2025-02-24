import cv2
import numpy as np
import time

def accumulate_contours(duration=5, video_source=2):
    """
    지정한 시간 동안 여러 프레임의 에지를 누적하여,
    최종 누적 마스크와 컨투어를 반환하는 함수입니다.

    Args:
        duration (int): 누적할 시간 (초)
        video_source (int or str): 비디오 소스 (기본값 0은 웹캠)

    Returns:
        clean_mask (ndarray): 후처리된 누적 에지 마스크
        contours (list): clean_mask에서 추출한 외형선(컨투어) 리스트
    """
    cap = cv2.VideoCapture(video_source)
    start_time = time.time()

    # 첫 프레임 읽기: 프레임 크기 결정
    ret, frame = cap.read()
    if not ret:
        print("비디오 소스를 읽어올 수 없습니다.")
        cap.release()
        return None, None

    # 프레임 크기에 맞는 단일 채널 누적 마스크 생성 (검은색)
    accumulated_mask = np.zeros_like(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 그레이스케일 변환 후, 가우시안 블러 적용
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # 누적 마스크에 현재 프레임의 에지 정보를 OR 연산으로 누적
        accumulated_mask = cv2.bitwise_or(accumulated_mask, edges)
        
        # 실시간 디버깅 용으로 현재 에지를 표시 (옵션)
        cv2.imshow("Real-time Edge Detection", edges)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # 지정한 시간(duration)이 지나면 누적 종료
        if time.time() - start_time > duration:
            break

    cap.release()
    cv2.destroyAllWindows()

    # 후처리: Morphological close 연산으로 간격 보정 및 노이즈 제거
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    clean_mask = cv2.morphologyEx(accumulated_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    # 누적 마스크에서 컨투어(외형선) 검출
    contours, _ = cv2.findContours(clean_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return clean_mask, contours

# accumulate_contours 함수 호출 (예: 5초 동안 누적)
accumulated_mask, final_contours = accumulate_contours(duration=5)

# 디버깅용: 누적된 에지 마스크를 표시하여 최종 결과 확인
cv2.imshow("Accumulated Edge Mask", accumulated_mask)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 컨투어를 컬러 이미지에 그려서 결과 확인 (원본에 가까운 형태로 시각화)
result_img = np.zeros((accumulated_mask.shape[0], accumulated_mask.shape[1], 3), dtype=np.uint8)
cv2.drawContours(result_img, final_contours, -1, (0, 255, 0), 2)
cv2.imshow("Final Contours", result_img)
cv2.waitKey(0)
cv2.destroyAllWindows()