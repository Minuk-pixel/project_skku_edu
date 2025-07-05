# manual_operation.py
# - 차선 기반 주행 실험용 코드 (방향키 수동 조종 버전)

import cv2
from utils.lane_detect import LaneDetect


lane_detector = LaneDetect()
print("[INFO] 방향키 ← ↑ → ↓ 로 수동 조향 테스트를 시작합니다. ESC로 종료")

while True:
    # 프레임 읽기 및 디버깅용 표시
    _ = lane_detector.compute_lane_control()

    key = cv2.waitKey(10) & 0xFF

    if key == 27:  # ESC
        lane_detector.send_serial_command("S")
        break
    elif key == ord('w') or key == 82:  # ↑ 전진
        lane_detector.send_serial_command("F")
    elif key == ord('s') or key == 84:  # ↓ 후진
        lane_detector.send_serial_command("B")
    elif key == ord('a') or key == 81:  # ← 좌회전
        lane_detector.send_serial_command("L")
    elif key == ord('d') or key == 83:  # → 우회전
        lane_detector.send_serial_command("R")
    elif key == ord(' '):  # 스페이스바 = 정지
        lane_detector.send_serial_command("S")

cv2.destroyAllWindows()
