# steering_fsm.py

class SteeringFSM:
    def __init__(self, Kp=0.03, dead_zone=10):
        self.state = "STRAIGHT"
        self.Kp = Kp
        self.dead_zone = dead_zone
        self.prev_command = "F"

    def update(self, cte):
        # P-Control 기반 조향 강도 계산
        control = self.Kp * cte

        # Dead zone 적용: 너무 작은 오차는 무시
        if abs(cte) < self.dead_zone:
            self.state = "STRAIGHT"
            command = "F"

        elif control > 0:
            self.state = "RIGHT"
            command = "R"

        elif control < 0:
            self.state = "LEFT"
            command = "L"

        else:
            command = "F"

        self.prev_command = command
        return command

    def reset(self):
        self.state = "STRAIGHT"
        self.prev_command = "F"
