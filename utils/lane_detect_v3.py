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

        self.image_width = 440  # 일반 카메라 프레임 기준 (수정 가능)
        self.image_height = 520
        self.image_center_x = self.image_width // 2
        self.bottom_y = self.image_height - 1

        self.prev_left_fit = np.array([0., 0.])
        self.prev_right_fit = np.array([0., 0.])
        self.alpha = 0.1  # EMA smoothing
        self.prev_cte = 0.0
        self.prev_heading = 0.0

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

        bev = cv2.warpPerspective(frame, self.transform_matrix, (440, 520))
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        points = np.argwhere(binary > 0)
        ys, xs = points[:, 0], points[:, 1]
        center_x = self.image_center_x

        left_x, left_y = xs[xs < center_x], ys[xs < center_x]
        right_x, right_y = xs[xs >= center_x], ys[xs >= center_x]

        #반대쪽 차선 픽셀이 섞이는 것 방지
        left_valid = (
                len(left_x) > 30 and
                np.max(left_x) < self.image_center_x - 30 and
                np.mean(left_x) < self.image_center_x - 40
        )

        right_valid = (
                len(right_x) > 30 and
                np.min(right_x) > self.image_center_x + 30 and
                np.mean(right_x) > self.image_center_x + 40
        )

        # left_fit = self.fit_line_ransac(left_x, left_y)
        # right_fit = self.fit_line_ransac(right_x, right_y)
        new_left_fit = self.fit_line_ransac(left_x, left_y)
        new_right_fit = self.fit_line_ransac(right_x, right_y)

        # after fitting
        if new_left_fit is not None and abs(new_left_fit[0]) < 0.05:
            print("[WARN] left slope too flat → rejected")
            new_left_fit = None

        if new_right_fit is not None and abs(new_right_fit[0]) < 0.05:
            print("[WARN] right slope too flat → rejected")
            new_right_fit = None

        # 급격한 기울기 변화 리젝 (차선 끊김 등 노이즈 대응)
        max_slope_change = 0.5  # 허용 가능한 최대 변화량 (조정 가능)

        if new_left_fit is not None and self.prev_left_fit is not None:
            if abs(new_left_fit[0] - self.prev_left_fit[0]) > max_slope_change:
                print("[REJECT] left slope changed too much")
                new_left_fit = None

        if new_right_fit is not None and self.prev_right_fit is not None:
            if abs(new_right_fit[0] - self.prev_right_fit[0]) > max_slope_change:
                print("[REJECT] right slope changed too much")
                new_right_fit = None

        # --- 스무딩 적용 (EMA) ---
        self.alpha = 0.1
        if new_left_fit is not None and abs(new_left_fit[0]) > 2.5:
            print("[REJECT] left_fit too steep")
            new_left_fit = None

        if new_right_fit is not None and abs(new_right_fit[0]) > 2.5:
            print("[REJECT] right_fit too steep")
            new_right_fit = None

        ploty = np.linspace(0, self.image_height - 1, self.image_height)
        left_fitx = new_left_fit[0] * ploty + new_left_fit[1] if new_left_fit is not None else None
        right_fitx = new_right_fit[0] * ploty + new_right_fit[1] if new_right_fit is not None else None

        overlay = bev.copy()

        if left_fitx is not None:
            valid_left = 0
            for i in range(len(ploty)):
                lx = int(left_fitx[i])
                py = int(ploty[i])
                if 0 <= lx < self.image_width:
                    valid_left += 1
                    cv2.circle(overlay, (lx, py), 1, (255, 0, 0), -1)
            print(f"[DEBUG] Left lane draw points: {valid_left}")

        if right_fitx is not None:
            valid_right = 0
            for i in range(len(ploty)):
                rx = int(right_fitx[i])
                py = int(ploty[i])
                if 0 <= rx < self.image_width:
                    valid_right += 1
                    cv2.circle(overlay, (rx, py), 1, (0, 0, 255), -1)
            print(f"[DEBUG] Right lane draw points: {valid_right}")

        cte_y = int(self.image_height * 0.6)  # 하단 말고 중간쯤에서 계산

        if new_left_fit is not None and new_right_fit is not None:
            left_x_pos = new_left_fit[0] * cte_y + new_left_fit[1]
            right_x_pos = new_right_fit[0] * cte_y + new_right_fit[1]
            lane_center = (left_x_pos + right_x_pos) / 2.0
            raw_cte = -(lane_center - self.image_center_x)
            raw_heading = (new_left_fit[0] + new_right_fit[0]) / 2.0

        elif new_left_fit is not None:
            lane_center = None
            raw_cte = 0.0  # CTE 안 씀
            raw_heading = new_left_fit[0]  # 왼쪽 기울기만 조향

        elif new_right_fit is not None:
            lane_center = None
            raw_cte = 0.0
            raw_heading = new_right_fit[0]  # 오른쪽 기울기만 조향

        else:
            raw_cte = 0.0
            raw_heading = 0.0


        cte = 0.5 * self.prev_cte + 0.5 * raw_cte
        heading = 0.8 * self.prev_heading + 0.2 * raw_heading

        self.prev_cte = cte
        self.prev_heading = heading

        print(cte)
        cv2.imshow("lowviewcam", frame)
        cv2.imshow("bev", bev)
        cv2.imshow("Lane Overlay", overlay)
        cv2.waitKey(1)

        return {
            'cte': cte,
            'heading': heading,
            'left_fit': new_left_fit,
            'right_fit': new_right_fit,
            'frame': frame,
            'overlay': overlay
        }

