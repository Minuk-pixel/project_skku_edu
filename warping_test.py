import cv2
import numpy as np

# ✅ 하향 카메라 인덱스
cap = cv2.VideoCapture(cv2.CAP_DSHOW + 2)

# ✅ 수동으로 지정한 source (원본 영상의 4점) & destination (BEV 평면)
source = np.float32([[108, 230], [0, 374], [470, 230], [575, 374]])
destination = np.float32([[0, 0], [0, 520], [440, 0], [440, 520]])

# ✅ 출력 사이즈는 destination 좌표 기준
output_width = 440
output_height = 520

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 카메라 프레임을 읽지 못했습니다.")
        break

    # 변환 행렬 구하기
    M = cv2.getPerspectiveTransform(source, destination)

    # 워핑 적용
    warped = cv2.warpPerspective(frame, M, (output_width, output_height))

    # BEV 이미지만 보여줌
    cv2.imshow("frame", frame)
    cv2.imshow("BEV (Warped)", warped)

    # 종료 조건
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
