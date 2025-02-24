import cv2
import numpy as np
import time

def get_contour_centroid(contour):
    """윤곽선의 중심 좌표를 구하는 함수"""
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)
    return None

def send_motor_signal(direction):
    """
    모터에 신호를 보내는 함수 (데모용)
    실제 구현에서는 GPIO나 시리얼 통신 등을 이용하여 모터를 제어합니다.
    direction: "forward" 혹은 "reverse"
    """
    print(f"Motor rotating: {direction}")

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        return

    # ROI 및 최소 면적 설정
    ROI_SCALE = 0.5           # 전체 프레임의 50% 크기의 중앙 영역을 ROI로 사용
    MIN_CONTOUR_AREA = 100    # 최소 윤곽선 면적 (잡음 제거용)
    TOLERANCE = 5             # ROI 중앙과 빨간 선 위치의 허용 오차 (픽셀)
    
    # 모터 제어 신호를 보낼 간격 (초)
    SIGNAL_INTERVAL = 1.0
    last_motor_command_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        # 프레임 크기 및 중앙 ROI 설정
        frame_h, frame_w = frame.shape[:2]
        roi_w = int(frame_w * ROI_SCALE)
        roi_h = int(frame_h * ROI_SCALE)
        x_start = (frame_w - roi_w) // 2
        y_start = (frame_h - roi_h) // 2
        roi = frame[y_start:y_start + roi_h, x_start:x_start + roi_w].copy()

        # ROI 내 전처리: 그레이스케일 변환 후 가우시안 블러 적용
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 임계값 처리 (대상과 배경에 맞게 threshold 값 조절)
        _, thresh = cv2.threshold(gray_blurred, 127, 255, cv2.THRESH_BINARY)

        # ROI 내 외곽선(윤곽선) 검출
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # ROI 중앙에 가까운 윤곽선 선택 (최소 면적 이상인 윤곽선만)
        roi_center = (roi_w // 2, roi_h // 2)
        selected_contour = None
        min_dist = float('inf')
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
                continue
            centroid = get_contour_centroid(cnt)
            if centroid is None:
                continue
            dist = np.linalg.norm(np.array(centroid) - np.array(roi_center))
            if dist < min_dist:
                min_dist = dist
                selected_contour = cnt

        red_line_y = None  # ROI 내부에서 빨간 선의 y 좌표 (행)

        if selected_contour is not None:
            # 윤곽선 좌표 배열: (N,1,2) → (N,2)
            pts = selected_contour.reshape(-1, 2)
            # 각 행(y 좌표)별 해당 행에 속하는 x 좌표들을 저장
            row_x_coords = {}
            for (x, y) in pts:
                if 0 <= y < roi_h:
                    row_x_coords.setdefault(y, []).append(x)
            # 각 행마다 x좌표의 최대값과 최소값 차이가 가장 큰 행 찾기
            max_diff = 0
            max_diff_row = None
            for y, x_list in row_x_coords.items():
                diff = max(x_list) - min(x_list)
                if diff > max_diff:
                    max_diff = diff
                    max_diff_row = y

            if max_diff_row is not None:
                red_line_y = max_diff_row
                # ROI 내부에 빨간 선 그리기
                cv2.line(roi, (0, red_line_y), (roi_w - 1, red_line_y), (0, 0, 255), 2)
                # 원본 프레임에 ROI 오프셋 반영하여 빨간 선 그리기
                cv2.line(frame, (x_start, y_start + red_line_y),
                         (x_start + roi_w - 1, y_start + red_line_y), (0, 0, 255), 2)
                cv2.putText(frame, f"Row: {red_line_y}", (x_start, y_start + red_line_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

            # 선택한 윤곽선을 녹색으로 표시 (디버깅용)
            cv2.drawContours(roi, [selected_contour], -1, (0, 255, 0), 2)

        # ROI 영역을 파란색 사각형으로 원본 프레임에 표시
        cv2.rectangle(frame, (x_start, y_start), (x_start + roi_w, y_start + roi_h), (255, 0, 0), 2)

        # 모터 제어: 빨간 선(red_line_y)이 ROI 중앙과의 차이에 따라 신호 전송
        if red_line_y is not None:
            roi_center_y = roi_h // 2
            diff = red_line_y - roi_center_y

            current_time = time.time()
            if current_time - last_motor_command_time >= SIGNAL_INTERVAL:
                # 허용 오차 내에 있으면 아무 신호도 보내지 않음
                if abs(diff) > TOLERANCE:
                    # 여기서 방향 결정은 시스템에 따라 달라집니다.
                    # 예를 들어, 빨간 선이 ROI 중앙보다 위에 있다면(diff < 0) 모터를 "reverse" 방향으로,
                    # 아래에 있다면(diff > 0) "forward" 방향으로 회전시킨다고 가정합니다.
                    if diff < 0:
                        direction = "reverse"
                    else:
                        direction = "forward"
                    send_motor_signal(direction)
                    last_motor_command_time = current_time
                else:
                    # 중앙에 근접하면 중앙 도달 메시지 출력
                    print("Red line centered. No motor command sent.")
                    last_motor_command_time = current_time

        # 결과 영상 출력
        cv2.imshow("Frame", frame)
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
