# main_time.py
# - 타임 트라이얼 주행용 메인 루프

from utils.lane_detect import LaneDetect
from utils.steering_fsm import SteeringFSM
import time

if __name__ == "__main__":
    detector = LaneDetect()
    fsm = SteeringFSM(Kp=0.01, dead_zone=10)

    print("[INFO] Time trial mode started. Press Ctrl+C to stop.")

    try:
        while True:
            result = detector.compute_lane_control()
            if result is None:
                continue

            cte = result['cte']
            cmd = fsm.update(cte)
            detector.send_serial_command(cmd)

            time.sleep(0.05)  # 20Hz 제어 주기 (필요 시 조정)

    except KeyboardInterrupt:
        print("[INFO] Time trial stopped by user.")
        detector.send_serial_command("S")
