# steering_fsm.py (기울기 기반 + soft-reset 적용)
import math

class SteeringFSM:
    def __init__(self, k=0.5, max_angle=20, velocity=0.25, pixel_to_meter=0.0014):
        self.k = k
        self.max_angle = max_angle
        self.velocity = velocity
        self.pixel_to_meter = pixel_to_meter

    def compute_steering_angle(self, cte_pixel, heading_slope):
        cte = cte_pixel * self.pixel_to_meter
        heading_angle = math.atan(heading_slope)

        if abs(cte_pixel) < 1e-3:
            # 한쪽 차선만 보이는 상황 → 기울기만 사용
            steer_rad = heading_angle * 0.8 # 필요시 1.2~1.5까지 튜닝 가능
        else:
            # 정상 Stanley 조향
            steer_rad = heading_angle * 0.8 + math.atan2(self.k * cte, self.velocity + 1e-5)

        steer_deg = math.degrees(steer_rad)
        steer_deg = max(-self.max_angle, min(self.max_angle, steer_deg))
        return steer_deg


