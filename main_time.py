# main_time.py
# - 타임 트라이얼 주행용 메인 루프

from utils.lane_detect_v3 import LaneDetectV3
from utils.steering_fsm import SteeringFSM
import time


if __name__ == "__main__":
    detector = LaneDetectV3()
    fsm = SteeringFSM(
        k=0.1,
        velocity=0.25,
        pixel_to_meter=0.0014  # ← 여기! 측정한 값 입력
    )
    detector.send_serial_command("C")

    print("[INFO] Time trial mode started. Press Ctrl+C to stop.")

    try:
        while True:
            result = detector.detect_lane_and_steering()
            if result is None:
                continue

            cte = result['cte']
            heading = result['heading']

            angle = fsm.compute_steering_angle(cte, heading)
            detector.send_serial_command(f"T{angle:.2f}\n")
            detector.send_serial_command("F")

            # detector.cam.arduino.write(f"T{angle:.2f}\n".encode())
            # detector.cam.arduino.write(b"F")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("[INFO] Time trial stopped by user.")
        detector.send_serial_command("S")
        # detector.cam.arduino.write(b"S")
