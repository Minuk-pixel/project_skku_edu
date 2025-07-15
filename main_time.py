# main_time.py
# - 타임 트라이얼 주행용 메인 루프

from utils.lane_detect_v3 import LaneDetectV3
from utils.steering_fsm import SteeringFSM
from utils.crosswalk import Crosswalk
import time


def change_left(detector):
    detector.send_serial_command("A")  # 왼쪽 기울이기
    time.sleep(2.0)
    detector.send_serial_command("D")  # 중앙으로 복귀
    time.sleep(2.0)
    detector.send_serial_command("C")


def change_right(detector):
    detector.send_serial_command("D")
    time.sleep(2.0)
    detector.send_serial_command("A")
    time.sleep(2.0)
    detector.send_serial_command("C")


if __name__ == "__main__":
    detector = LaneDetectV3()
    crosswalk_detector = Crosswalk()
    fsm = SteeringFSM(
        k=0.01,
        velocity=0.25,
        pixel_to_meter=0.0014  # ← 여기! 측정한 값 입력
    )
    detector.send_serial_command("C")

    print("[INFO] Time trial mode started. Press Ctrl+C to stop.")
    detector.current_target = "RIGHT"

    try:
        while True:
            result = detector.detect_lane_and_steering()
            is_crosswalk = crosswalk_detector.crosswalk()
            crosswalk_cooldown = 0

            if result is None:
                continue

            if crosswalk_cooldown > 0:
                crosswalk_cooldown -= 1
                continue

            cte = result['cte']
            heading = result['heading']

            angle = fsm.compute_steering_angle(cte, heading)
            detector.send_serial_command(f"T{angle:.2f}\n")
            detector.send_serial_command("F")

            if is_crosswalk and detector.current_target == "RIGHT":
                print("✅ 횡단보도 감지됨!")
                detector.set_lane("LEFT")
                change_left(detector)
                crosswalk_cooldown = 100  # 약 1초 정도 대기 (100 x 0.01s 루프 딜레이)
                print("차선변경 완료!")
                continue
            elif is_crosswalk and detector.current_target == "LEFT":
                print("✅ 횡단보도 감지됨!")
                detector.set_lane("RIGHT")
                change_right(detector)
                crosswalk_cooldown = 100  # 약 1초 정도 대기 (100 x 0.01s 루프 딜레이)
                print("차선변경 완료!")
                continue

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("[INFO] Time trial stopped by user.")
        detector.send_serial_command("S")
        # detector.cam.arduino.write(b"S")
