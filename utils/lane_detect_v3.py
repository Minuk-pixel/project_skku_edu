import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

import cv2, numpy as np, time

from sklearn.linear_model import RANSACRegressor
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression

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

        # self.image_width = 440  # 일반 카메라 프레임 기준 (수정 가능)
        # self.image_height = 520
        # self.image_center_x = self.image_width // 2

        self.roi_top = 250
        self.roi_bottom = 570
        self.roi_left = 50
        self.roi_right = 550
        self.image_width = self.roi_right - self.roi_left
        self.image_height = self.roi_bottom - self.roi_top
        self.image_center_x = self.image_width // 2

        self.bottom_y = self.image_height - 1

        self.prev_left_fit = np.array([0, 0])
        self.prev_right_fit = np.array([0, 0])
        self.alpha = 0.2  # EMA smoothing
        self.prev_cte = 0.0
        self.prev_heading = 0.0

    def fit_line_ransac(self, x, y):
        if len(x) < 50:
            print("[WARN] too few points for fitting")
            return None
        X = np.array(x).reshape(-1, 1)
        Y = np.array(y)

        model = RANSACRegressor(
            estimator=LinearRegression(),  # sklearn ≥ 0.24일 경우
            residual_threshold=5.0,
            max_trials=50
        )

        try:
            model.fit(X, Y)
            slope = model.estimator_.coef_[0]
            intercept = model.estimator_.intercept_
        except Exception as e:
            print(f"[ERROR] RANSAC fitting failed: {e}")
            return None

        return np.array([slope, intercept])

    def send_serial_command(self, command):
        if self.ser is not None and self.ser.is_open:
            self.ser.write((command).encode())
            print(f"[TX] {command}")
        else:
            print("[WARN] Serial not connected.")

    def compute_heading(self, fit):
        return fit[0]  # heading = slope

    def detect_lane_and_steering(self):
        ret, frame = self.cap1.read()
        if not ret:
            print("[WARN] Camera read failed")
            return None

        roi = frame[self.roi_top:self.roi_bottom, self.roi_left:self.roi_right]  # 아래에서 범위 지정
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        points = np.argwhere(binary > 0)
        ys, xs = points[:, 0], points[:, 1]
        center_x = self.image_center_x

        left_x, left_y = xs[xs < center_x], ys[xs < center_x]
        right_x, right_y = xs[xs >= center_x], ys[xs >= center_x]


        # left_fit = self.fit_line_ransac(left_x, left_y)
        # right_fit = self.fit_line_ransac(right_x, right_y)
        new_left_fit = self.fit_line_ransac(left_x, left_y)
        new_right_fit = self.fit_line_ransac(right_x, right_y)

        # after fitting
        # if new_left_fit is not None and abs(new_left_fit[0]) < 0.05:
        #     print("[WARN] left slope too flat → rejected")
        #     new_left_fit = None
        #
        # if new_right_fit is not None and abs(new_right_fit[0]) < 0.05:
        #     print("[WARN] right slope too flat → rejected")
        #     new_right_fit = None

        # --- 스무딩 적용 (EMA) ---
        self.alpha = 0.2
        if new_left_fit is not None:
            if new_left_fit is not None and self.prev_left_fit.shape != new_left_fit.shape:
                self.prev_left_fit = np.zeros_like(new_left_fit)
            left_fit = self.alpha * self.prev_left_fit + (1 - self.alpha) * new_left_fit
            self.prev_left_fit = left_fit
        else:
            left_fit = self.prev_left_fit

        if new_right_fit is not None:
            if new_right_fit is not None and self.prev_right_fit.shape != new_right_fit.shape:
                self.prev_right_fit = np.zeros_like(new_right_fit)

            right_fit = self.alpha * self.prev_right_fit + (1 - self.alpha) * new_right_fit
            self.prev_right_fit = right_fit
        else:
            right_fit = self.prev_right_fit

        # if left_fit is not None:
        #     left_fit = alpha * self.prev_left_fit + (1 - alpha) * left_fit
        #     self.prev_left_fit = left_fit
        # else:
        #     left_fit = self.prev_left_fit
        #
        # if right_fit is not None:
        #     right_fit = alpha * self.prev_right_fit + (1 - alpha) * right_fit
        #     self.prev_right_fit = right_fit
        # else:
        #     right_fit = self.prev_right_fit

        ploty = np.linspace(0, self.image_height - 1, self.image_height)
        # Solve for x instead: x = (y - b) / a
        left_fitx = (ploty - left_fit[1]) / left_fit[0] if left_fit is not None else None
        right_fitx = (ploty - right_fit[1]) / right_fit[0] if right_fit is not None else None

        # overlay = bev.copy()
        # for i in range(len(ploty)):
        #     lx = int(left_fitx[i])
        #     rx = int(right_fitx[i])
        #     py = int(ploty[i])
        #
        #     if 0 <= lx < self.image_width:
        #         cv2.circle(overlay, (lx, py), 1, (255, 0, 0), -1)
        #     if 0 <= rx < self.image_width:
        #         cv2.circle(overlay, (rx, py), 1, (0, 0, 255), -1)

        overlay = roi.copy()

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

        cte_y = int(self.image_height * 0.6)  # 조향 기준이 될 y 지점

        if left_fit is not None and right_fit is not None:
            raw_heading = (self.compute_heading(left_fit) + self.compute_heading(right_fit)) / 2.0
        elif left_fit is not None:
            raw_heading = self.compute_heading(left_fit)
        elif right_fit is not None:
            raw_heading = self.compute_heading(right_fit)
        else:
            raw_heading = 0.0

        # left_x_pos = left_fit[0] * self.bottom_y + left_fit[1]
        # right_x_pos = right_fit[0] * self.bottom_y + right_fit[1]
        # lane_center = (left_x_pos + right_x_pos) / 2.0
        #
        # # --- CTE 및 Heading 계산 + 스무딩 ---
        # raw_cte = -(lane_center - self.image_center_x)
        # raw_heading = (left_fit[0] + right_fit[0]) / 2.0

        raw_cte = 0.0  # ← 고정값으로 둠
        cte = 0.5 * self.prev_cte + 0.5 * raw_cte
        heading = 0.8 * self.prev_heading + 0.2 * raw_heading

        self.prev_cte = cte
        self.prev_heading = heading

        print(cte)
        cv2.imshow("lowviewcam", frame)
        cv2.imshow("roi", roi)
        cv2.imshow("Lane Overlay", overlay)
        cv2.waitKey(1)

        return {
            'cte': cte,
            'heading': heading,
            'left_fit': left_fit,
            'right_fit': right_fit,
            'frame': frame,
            'overlay': overlay
        }

