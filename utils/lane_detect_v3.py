import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

import cv2, numpy as np, time

from sklearn.linear_model import RANSACRegressor
import Function_Library as fl

class LaneDetectV3:
    def __init__(self):
        self.cam = fl.libCAMERA()
        self.cap1 = cv2.VideoCapture(cv2.CAP_DSHOW + 2)  # Downward camera

        # 아두이노 시리얼 연결
        self.ardu = fl.libARDUINO()
        # 기존 (리눅스용)
        # self.ser = self.ardu.init('/dev/ttyUSB0', 9600)
        # 수정 (Windows용)
        self.ser = self.ardu.init('COM3', 9600)  # COM3 대신 당신 PC의 실제 포트 번호로

        print("[INFO] Arduino serial connected.")

        self.source = np.float32([[128, 304], [0, 440], [465, 304], [577, 440]])
        self.destination = np.float32([[0, 0], [0, 520], [440, 0], [440, 520]])
        self.transform_matrix = cv2.getPerspectiveTransform(self.source, self.destination)

        self.prev_heading_valid = 0.0  # fallback용 직전 유효 heading
        self.max_heading_change = 0.15  # 프레임당 heading 변화 최대값
        self.max_slope_jump = 0.3  # sudden jump reject 기준

        self.image_width = 440  # 일반 카메라 프레임 기준 (수정 가능)
        self.image_height = 520
        self.image_center_x = self.image_width // 2
        self.bottom_y = self.image_height - 1

        self.prev_left_fit = np.array([0., 0.])
        self.prev_right_fit = np.array([0., 0.])
        self.alpha = 0.1  # EMA smoothing
        self.prev_cte = 0.0
        self.prev_heading = 0.0
        self.current_target = "RIGHT"

    def fit_line_ransac(self, x, y):
        if len(x) < 10:
            return None
        X = np.array(y).reshape(-1, 1)
        Y = np.array(x)
        model = RANSACRegressor()

        model.fit(X, Y)
        slope = model.estimator_.coef_[0]
        intercept = model.estimator_.intercept_
        return np.array([slope, intercept])

    def send_serial_command(self, command):
        if self.ser is not None and self.ser.is_open:
            self.ser.write((command).encode())
            print(f"[TX] {command}")
        else:
            print("[WARN] Serial not connected.")

    def set_lane(self, lane: str):
        assert lane in ['LEFT', 'RIGHT']
        self.current_target = lane
        print(f"[INFO] Lane target changed to {lane}")

    def detect_lane_and_steering(self):
        ret, frame = self.cap1.read()
        if not ret:
            print("[WARN] Camera read failed")
            return None

        # 1. BEV 변환 및 색상 필터링 (흰 실선만)
        bev = cv2.warpPerspective(frame, self.transform_matrix, (440, 520))
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 40, 255])  # 점선 제거 위해 S 값 조절 가능
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 2. 모폴로지로 점선 제거 (선택)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 3. 실선 픽셀 좌표 수집
        points = np.argwhere(binary > 0)
        if len(points) == 0:
            print("[FALLBACK] No line pixels detected.")
            raw_heading = self.prev_heading_valid
            raw_cte = 0.0
            overlay = bev.copy()
        else:
            ys, xs = points[:, 0], points[:, 1]

            # 4. 단일 실선에 대해 RANSAC fitting
            fit = self.fit_line_ransac(xs, ys)

            if fit is None or abs(fit[0]) < 0.05 or abs(fit[0]) > 2.5:
                print("[REJECT] Invalid or unstable fit → fallback")
                raw_heading = self.prev_heading_valid
                raw_cte = 0.0
            else:
                current_heading = fit[0]
                delta = abs(current_heading - self.prev_heading)

                if delta > self.max_slope_jump:
                    print(f"[REJECT] Heading jump too large: {delta:.3f}")
                    current_heading = self.prev_heading_valid
                else:
                    self.prev_heading_valid = current_heading

                raw_heading = current_heading
                raw_cte = 0.0  # 실선 하나만 쓰므로 CTE 무의미

            # 5. 시각화
            ploty = np.linspace(0, self.image_height - 1, self.image_height)
            fitx = fit[0] * ploty + fit[1] if fit is not None else None

            overlay = bev.copy()
            if fitx is not None:
                for i in range(len(ploty)):
                    fx = int(fitx[i])
                    fy = int(ploty[i])
                    if 0 <= fx < self.image_width:
                        cv2.circle(overlay, (fx, fy), 1, (0, 255, 0), -1)
            else:
                overlay = bev.copy()

        # 6. heading smoothing + delta 제한
        cte = 0.0
        delta_heading = raw_heading - self.prev_heading
        delta_heading = np.clip(delta_heading, -self.max_heading_change, self.max_heading_change)
        heading = self.prev_heading + delta_heading

        # 7. 상태 업데이트
        self.prev_cte = cte
        self.prev_heading = heading

        print(f"[STATE] Target={self.current_target}, Heading={heading:.3f}")
        cv2.imshow("lowviewcam", frame)
        cv2.imshow("bev", bev)
        cv2.imshow("Lane Overlay", overlay)
        cv2.waitKey(1)

        return {
            'cte': cte,
            'heading': heading,
            'fit': fit,
            'frame': frame,
            'overlay': overlay
        }

