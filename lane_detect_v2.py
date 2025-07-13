# lane_detect_v2.py
# - 슬라이딩 윈도우 제거, edge 기반 차선 피팅으로 CTE/Heading 추정

import cv2
import numpy as np
import Function_Library as fl

class LaneDetect:
    def __init__(self):
        self.cam = fl.libCAMERA()
        self.cam.capnum = 2
        self.ardu = fl.libARDUINO()
        self.ser = self.ardu.init('COM3', 9600)
        print("[INFO] Arduino serial connected.")

        self.cap1 = cv2.VideoCapture(cv2.CAP_DSHOW + 1)
        self.cap2 = cv2.VideoCapture(cv2.CAP_DSHOW + 2)

        self.source = np.float32([[91, 173], [0, 286], [516, 173], [614, 286]])
        self.destination = np.float32([[0, 0], [0, 520], [440, 0], [440, 520]])
        self.transform_matrix = cv2.getPerspectiveTransform(self.source, self.destination)

        self.warped_center_x = 220
        self.warped_image_height = 520

        self.prev_left_fit = np.array([0., 0.])
        self.prev_right_fit = np.array([0., 0.])
        self.fit_smoothing_factor = 0.8

        self.fallback_active = False

    def send_serial_command(self, command):
        if self.ser is not None and self.ser.is_open:
            self.ser.write((command).encode())
            print(f"[TX] {command}")
        else:
            print("[WARN] Serial not connected.")

    def warpping(self, image):
        return cv2.warpPerspective(image, self.transform_matrix, (440, 520))

    def read_downward_camera(self):
        result = self.cam.camera_read(self.cap1, self.cap2)

        if len(result) != 4:
            print("[WARN] camera_read returned unexpected data:", result)
            return False, None

        ret2, frame2 = result[2], result[3]
        if not ret2 or frame2 is None:
            print("[WARN] Downward camera read failed.")
            return False, None

        return True, frame2

    def edge_based_lane_detect(self, bev_img, show_debug=False):
        gray = cv2.cvtColor(bev_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        binary = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 15, 10
        )

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        height, width = binary.shape
        center_x = width // 2

        left_x, left_y, right_x, right_y = [], [], [], []

        for cnt in contours:
            if cv2.contourArea(cnt) < 50:
                continue
            for point in cnt:
                x, y = point[0]
                if x < center_x:
                    left_x.append(x)
                    left_y.append(y)
                else:
                    right_x.append(x)
                    right_y.append(y)

        out_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        if len(left_x) > 30:
            left_fit = np.polyfit(left_y, left_x, 1)
            self.prev_left_fit = left_fit
        else:
            left_fit = self.prev_left_fit

        if len(right_x) > 30:
            right_fit = np.polyfit(right_y, right_x, 1)
            self.prev_right_fit = right_fit
        else:
            right_fit = self.prev_right_fit

        ploty = np.linspace(0, height - 1, height)
        left_fitx = left_fit[0] * ploty + left_fit[1]
        right_fitx = right_fit[0] * ploty + right_fit[1]

        for i in range(len(ploty)):
            lx, rx, py = int(left_fitx[i]), int(right_fitx[i]), int(ploty[i])
            if 0 <= lx < width:
                cv2.circle(out_img, (lx, py), 1, (255, 0, 0), -1)
            if 0 <= rx < width:
                cv2.circle(out_img, (rx, py), 1, (0, 0, 255), -1)

        draw_info = {
            'left_fitx': left_fitx,
            'right_fitx': right_fitx,
            'ploty': ploty,
            'left_fit': left_fit,
            'right_fit': right_fit
        }

        return draw_info, out_img

    def compute_lane_control(self):
        ret2, frame2 = self.read_downward_camera()
        if not ret2:
            return None

        bev = self.warpping(frame2)
        filtered = self.cam.color_filtering(bev)

        draw_info, windows = self.edge_based_lane_detect(filtered, show_debug=True)

        bottom_y = self.warped_image_height - 1
        left_detected = len(draw_info['left_fitx']) > 30 and not np.all(draw_info['left_fit'] == 0)
        right_detected = len(draw_info['right_fitx']) > 30 and not np.all(draw_info['right_fit'] == 0)

        if left_detected and right_detected:
            self.fallback_active = False
            left_x = draw_info['left_fit'][0] * bottom_y + draw_info['left_fit'][1]
            right_x = draw_info['right_fit'][0] * bottom_y + draw_info['right_fit'][1]
            lane_center = (left_x + right_x) / 2.0
            cte = lane_center - self.warped_center_x
            heading = (draw_info['left_fit'][0] + draw_info['right_fit'][0]) / 2.0
        elif left_detected:
            self.fallback_active = False
            left_x = draw_info['left_fit'][0] * bottom_y + draw_info['left_fit'][1]
            lane_center = left_x + 100 if left_x < self.warped_center_x else left_x - 100
            cte = lane_center - self.warped_center_x
            heading = draw_info['left_fit'][0]
        elif right_detected:
            self.fallback_active = False
            right_x = draw_info['right_fit'][0] * bottom_y + draw_info['right_fit'][1]
            lane_center = right_x - 100 if right_x > self.warped_center_x else right_x + 100
            cte = lane_center - self.warped_center_x
            heading = draw_info['right_fit'][0]
        else:
            self.fallback_active = True
            cte = 0.0
            heading = 0.0

        print(f"[DEBUG] CTE (px): {cte:.2f}, Heading (slope): {heading:.4f}")

        cv2.imshow("lowviewcam", frame2)
        cv2.imshow("BEV", bev)
        cv2.imshow("White Filter", filtered)
        cv2.imshow("Windows", windows)
        cv2.waitKey(1)

        return draw_info, cte, heading, self.fallback_active
