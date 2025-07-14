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

        self.source = np.float32([[105, 350], [49, 406], [570, 350], [632, 406]])
        self.destination = np.float32([[0, 0], [0, 520], [440, 0], [440, 520]])
        self.transform_matrix = cv2.getPerspectiveTransform(self.source, self.destination)

        self.image_width = 440  # 일반 카메라 프레임 기준 (수정 가능)
        self.image_height = 520
        self.image_center_x = self.image_width // 2
        self.bottom_y = self.image_height - 1

        self.prev_left_fit = np.array([0., 0.])
        self.prev_right_fit = np.array([0., 0.])
        self.last_update_time = 0
        self.update_interval = 0.1  # 0.5초마다 업데이트
        self.alpha = 0.8  # EMA smoothing
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

    def detect_lane_and_steering(self):
        now = time.time()
        update = (now - self.last_update_time > self.update_interval)

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
                np.max(left_x) < self.image_center_x * 0.95 and
                np.mean(left_x) < self.image_center_x * 0.9
        )

        right_valid = (
                len(right_x) > 30 and
                np.min(right_x) > self.image_center_x * 1.05 and
                np.mean(right_x) > self.image_center_x * 1.1
        )

        # left_fit = self.fit_line_ransac(left_x, left_y)
        # right_fit = self.fit_line_ransac(right_x, right_y)
        if update:
            self.last_update_time = now
            new_left_fit = self.fit_line_ransac(left_x, left_y) if left_valid else None
            new_right_fit = self.fit_line_ransac(right_x, right_y) if right_valid else None
        else:
            new_left_fit = None
            new_right_fit = None

        # --- 스무딩 적용 (EMA) ---
        self.alpha = 0.8
        if new_left_fit is not None:
            left_fit = self.alpha * self.prev_left_fit + (1 - self.alpha) * new_left_fit
            self.prev_left_fit = left_fit
        else:
            left_fit = self.prev_left_fit

        if new_right_fit is not None:
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
        left_fitx = left_fit[0] * ploty + left_fit[1] if left_fit is not None else None
        right_fitx = right_fit[0] * ploty + right_fit[1] if right_fit is not None else None

        overlay = bev.copy()
        for i in range(len(ploty)):
            lx = int(left_fitx[i])
            rx = int(right_fitx[i])
            py = int(ploty[i])

            if 0 <= lx < self.image_width:
                cv2.circle(overlay, (lx, py), 1, (255, 0, 0), -1)
            if 0 <= rx < self.image_width:
                cv2.circle(overlay, (rx, py), 1, (0, 0, 255), -1)

        cte_y = int(self.image_height * 0.6)  # 하단 말고 중간쯤에서 계산

        if left_fit is not None and right_fit is not None:
            left_x_pos = left_fit[0] * cte_y + left_fit[1]
            right_x_pos = right_fit[0] * cte_y + right_fit[1]
            lane_center = (left_x_pos + right_x_pos) / 2.0
            raw_cte = -(lane_center - self.image_center_x)
            raw_heading = (left_fit[0] + right_fit[0]) / 2.0

        elif left_fit is not None:
            lane_center = None
            raw_cte = 0.0  # CTE 안 씀
            raw_heading = left_fit[0]  # 왼쪽 기울기만 조향

        elif right_fit is not None:
            lane_center = None
            raw_cte = 0.0
            raw_heading = right_fit[0]  # 오른쪽 기울기만 조향

        else:
            raw_cte = 0.0
            raw_heading = 0.0

        # left_x_pos = left_fit[0] * self.bottom_y + left_fit[1]
        # right_x_pos = right_fit[0] * self.bottom_y + right_fit[1]
        # lane_center = (left_x_pos + right_x_pos) / 2.0
        #
        # # --- CTE 및 Heading 계산 + 스무딩 ---
        # raw_cte = -(lane_center - self.image_center_x)
        # raw_heading = (left_fit[0] + right_fit[0]) / 2.0

        cte = 0.8 * self.prev_cte + 0.2 * raw_cte
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
            'left_fit': left_fit,
            'right_fit': right_fit,
            'frame': frame,
            'overlay': overlay
        }

