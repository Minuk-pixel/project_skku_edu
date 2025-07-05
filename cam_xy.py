import cv2

# 전역 변수로 좌표 저장
clicked_points = []

# 마우스 콜백 함수 정의
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_points.append((x, y))
        print(f"📍 Clicked Point: ({x}, {y})")
        cv2.circle(param, (x, y), 5, (0, 0, 255), -1)

# 비디오 캡처 (예: 하향 카메라가 cap 2번)
cap = cv2.VideoCapture(cv2.CAP_DSHOW + 2)

cv2.namedWindow("Downward Camera")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 프레임을 읽지 못했습니다.")
        break

    display = frame.copy()
    cv2.setMouseCallback("Downward Camera", mouse_callback, display)

    # 이미 찍은 점들 표시
    for pt in clicked_points:
        cv2.circle(display, pt, 5, (0, 0, 255), -1)

    cv2.imshow("Downward Camera", display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# 좌표 출력 마무리
print("\n✅ 최종 선택한 좌표 목록:")
for i, pt in enumerate(clicked_points):
    print(f"{i+1}: {pt}")
