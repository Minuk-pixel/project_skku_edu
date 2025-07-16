# utils/lane_detect.py
# - BEV 변환, 흰색 차선 필터링, 슬라이딩 윈도우 기반 차선 검출 클래스
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

import cv2, numpy as np, time
import Function_Library as fl

class LaneDetect:
    def __init__(self):
        # 카메라 설정 (듀얼 카메라 기준)
        self.cam = fl.libCAMERA()
        self.cam.capnum = 2   # ← camera_read에서 이 값 없으면 아무 것도 안 읽음

        # 아두이노 시리얼 연결
        self.ardu = fl.libARDUINO()
        # 기존 (리눅스용)
        # self.ser = self.ardu.init('/dev/ttyUSB0', 9600)
        # 수정 (Windows용)
        self.ser = self.ardu.init('COM3', 9600)  # COM3 대신 당신 PC의 실제 포트 번호로

        print("[INFO] Arduino serial connected.")

        #self.cap1, self.cap2 = self.cam.initial_setting(capnum=2)
        # 카메라 인덱스 확인 후 적절한 번호로 수정
        self.cap1 = cv2.VideoCapture(cv2.CAP_DSHOW + 1)
        self.cap2 = cv2.VideoCapture(cv2.CAP_DSHOW + 2)

        # 이미지 BEV 변환용 파라미터 (소스/타겟 좌표)
        # self.source = np.float32([[91, 173], [0, 286], [516, 173], [614, 286]])
        # self.destination = np.float32([[0, 0], [0, 520], [440, 0], [440, 520]])
        # self.transform_matrix = cv2.getPerspectiveTransform(source, destination)
        self.source = np.float32([[120, 155], [0, 349], [506, 155], [631, 349]])
        self.destination = np.float32([[0, 0], [0, 520], [440, 0], [440, 520]])
        self.transform_matrix = cv2.getPerspectiveTransform(self.source, self.destination)

        # BEV 이미지 중심 좌표
        self.warped_center_x = 220
        self.warped_image_height = 520

        self.prev_left_base = None
        self.prev_right_base = None
        self.base_smoothing_factor = 0.7
        self.prev_left_fit = np.array([0., 0.])
        self.prev_right_fit = np.array([0., 0.])
        self.fit_smoothing_factor = 0.8

    def get_lane_curve(self, binary):
        # === 1. Morphology: 끊긴 선 연결 ===
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # === 2. 흰 점 좌표 수집 ===
        points = np.argwhere(binary > 0)
        ys = points[:, 0]
        xs = points[:, 1]

        # 왼쪽 / 오른쪽 분리 기준 (가운데 x값)
        center_x = binary.shape[1] // 2
        left_x, left_y = xs[xs < center_x], ys[xs < center_x]
        right_x, right_y = xs[xs >= center_x], ys[xs >= center_x]

        # === 3. polyfit 안정화 조건 ===
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

        # === 4. 결과 디버깅 (직선 시각화 등) ===
        # 예: slope 기반 중앙 steering 계산 등 수행 가능
        return left_fit, right_fit

    def send_serial_command(self, command):
        if self.ser is not None and self.ser.is_open:
            self.ser.write((command).encode())
            print(f"[TX] {command}")
        else:
            print("[WARN] Serial not connected.")

    def warpping(self, image):
        return cv2.warpPerspective(image, self.transform_matrix, (440, 520))

    def color_filter_old(self, image):
        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

        # 튜닝된 흰색 범위: H 전체, L은 160 이상, S는 0~120 정도 허용
        lower_white = np.array([0, 180, 0])
        upper_white = np.array([180, 255, 130])

        white_mask = cv2.inRange(hls, lower_white, upper_white)
        masked = cv2.bitwise_and(image, image, mask=white_mask)
        return masked

    def color_filter(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        binary = cv2.adaptiveThreshold(
            blurred, 255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY_INV,
            blockSize=15,
            C=10
        )

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        #binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.dilate(binary, kernel, iterations=1)

        return cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

    def plothistogram(self, image):
        histogram = np.sum(image[:, :], axis=0)
        midpoint = np.int_(histogram.shape[0] / 2)
        current_left_base = np.argmax(histogram[:midpoint])
        current_right_base = np.argmax(histogram[midpoint:]) + midpoint
        MIN_PEAK_HEIGHT = 1000

        # 왼쪽 차선 베이스 계산
        leftbase = current_left_base
        if histogram[current_left_base] < MIN_PEAK_HEIGHT:
            leftbase = self.prev_left_base if self.prev_left_base is not None else midpoint // 2
        else:
            if self.prev_left_base is not None and abs(current_left_base - self.prev_left_base) <= 100:
                leftbase = int(self.base_smoothing_factor * self.prev_left_base +
                               (1 - self.base_smoothing_factor) * current_left_base)

        # 오른쪽 차선 베이스 계산
        rightbase = current_right_base
        if histogram[current_right_base] < MIN_PEAK_HEIGHT:
            rightbase = self.prev_right_base if self.prev_right_base is not None else midpoint + midpoint // 2
        else:
            if self.prev_right_base is not None and abs(current_right_base - self.prev_right_base) <= 100:
                rightbase = int(self.base_smoothing_factor * self.prev_right_base +
                                (1 - self.base_smoothing_factor) * current_right_base)

        self.prev_left_base = leftbase
        self.prev_right_base = rightbase
        return leftbase, rightbase

    # def sliding_window_search(self, binary, left_base, right_base):
    #     nwindows = 30
    #     window_height = np.int_(binary.shape[0] / nwindows)
    #     nonzero = binary.nonzero()
    #     nonzeroy, nonzerox = nonzero[0], nonzero[1]
    #     margin, minpix = 80, 30
    #     left_current, right_current = left_base, right_base
    #     left_lane_inds, right_lane_inds = [], []
    #
    #     # 디버깅용 출력 이미지 생성 (3채널 컬러)
    #     out_img = np.dstack((binary, binary, binary)) * 255
    #
    #     for window in range(nwindows):
    #         win_y_low = binary.shape[0] - (window + 1) * window_height
    #         win_y_high = binary.shape[0] - window * window_height
    #         win_xleft_low = left_current - margin
    #         win_xleft_high = left_current + margin
    #         win_xright_low = right_current - margin
    #         win_xright_high = right_current + margin
    #
    #         # 사각형 시각화
    #         cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
    #         cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
    #
    #         good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
    #                           (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
    #         good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
    #                            (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
    #
    #         if len(good_left_inds) > minpix:
    #             left_current = np.int_(np.mean(nonzerox[good_left_inds]))
    #         if len(good_right_inds) > minpix:
    #             right_current = np.int_(np.mean(nonzerox[good_right_inds]))
    #
    #         left_lane_inds.append(good_left_inds)
    #         right_lane_inds.append(good_right_inds)
    #
    #     left_lane_inds = np.concatenate(left_lane_inds)
    #     right_lane_inds = np.concatenate(right_lane_inds)
    #     leftx = nonzerox[left_lane_inds]
    #     lefty = nonzeroy[left_lane_inds]
    #     rightx = nonzerox[right_lane_inds]
    #     righty = nonzeroy[right_lane_inds]
    #
    #     left_fit = np.polyfit(lefty, leftx, 1) if len(leftx) > 0 else [0, 0]
    #     right_fit = np.polyfit(righty, rightx, 1) if len(rightx) > 0 else [0, 0]
    #
    #     # 피팅된 선 그리기
    #     ploty = np.linspace(0, binary.shape[0] - 1, binary.shape[0])
    #     left_fitx = left_fit[0] * ploty + left_fit[1]
    #     right_fitx = right_fit[0] * ploty + right_fit[1]
    #
    #     for i in range(len(ploty)):
    #         cv2.circle(out_img, (int(left_fitx[i]), int(ploty[i])), 1, (255, 0, 0), -1)
    #         cv2.circle(out_img, (int(right_fitx[i]), int(ploty[i])), 1, (0, 0, 255), -1)
    #
    #     return left_fit, right_fit, out_img
    def slide_window_search(self, binary, left_base, right_base):
        nwindows = 30
        window_height = np.int_(binary.shape[0] / nwindows)
        nonzero = binary.nonzero()
        nonzeroy, nonzerox = nonzero[0], nonzero[1]
        margin, minpix = 80, 30
        left_current, right_current = left_base, right_base
        left_lane_inds, right_lane_inds = [], []

        out_img = np.dstack((binary, binary, binary)) * 255

        for window in range(nwindows):
            win_y_low = binary.shape[0] - (window + 1) * window_height
            win_y_high = binary.shape[0] - window * window_height
            win_xleft_low = left_current - margin
            win_xleft_high = left_current + margin
            win_xright_low = right_current - margin
            win_xright_high = right_current + margin

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            if len(good_left_inds) > minpix:
                left_current = np.int_(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                right_current = np.int_(np.mean(nonzerox[good_right_inds]))

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # 차선 직선 피팅 및 스무딩
        if len(leftx) > 20:
            left_fit = np.polyfit(lefty, leftx, 1)
            self.prev_left_fit = self.fit_smoothing_factor * self.prev_left_fit + \
                                 (1 - self.fit_smoothing_factor) * left_fit
        else:
            left_fit = self.prev_left_fit

        if len(rightx) > 20:
            right_fit = np.polyfit(righty, rightx, 1)
            self.prev_right_fit = self.fit_smoothing_factor * self.prev_right_fit + \
                                  (1 - self.fit_smoothing_factor) * right_fit
        else:
            right_fit = self.prev_right_fit

        ploty = np.linspace(0, binary.shape[0] - 1, binary.shape[0])
        left_fitx = left_fit[0] * ploty + left_fit[1]
        right_fitx = right_fit[0] * ploty + right_fit[1]

        for i in range(len(ploty)):
            cv2.circle(out_img, (int(left_fitx[i]), int(ploty[i])), 1, (255, 255, 0), -1)
            cv2.circle(out_img, (int(right_fitx[i]), int(ploty[i])), 1, (255, 255, 0), -1)

        return {'left_fitx': left_fitx, 'right_fitx': right_fitx,
                'ploty': ploty, 'left_fit': left_fit, 'right_fit': right_fit}, out_img

    def read_downward_camera(self):
        result = self.cam.camera_read(self.cap1, self.cap2)

        if len(result) != 4:
            print("[WARN] camera_read returned unexpected data:", result)
            return False, None

        ret2, frame2 = result[2], result[3]  # cap2 기준 (하향카메라)
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
        blurred_img = cv2.GaussianBlur(bev, (0, 0), 1)
        filtered = self.color_filter(blurred_img)
        # kernel = np.ones((5, 5), np.uint8)
        # filtered = cv2.morphologyEx(filtered, cv2.MORPH_CLOSE, kernel, iterations=2)
        gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)

        contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue

            x, y, w, h = cv2.boundingRect(c)
            aspect_ratio = float(w) / h if h != 0 else 0

            # 박스 윤곽 제거 조건들
            # if (aspect_ratio > 5 or area / perimeter < 2.8) or (w > 100 and h < 30):  # 1.5, 1.8, 2.0  area / perimeter < 2.0 이게 면적조건
            #     cv2.drawContours(gray, [c], -1, 0, -1)
            if area / perimeter < 2.7 or  (w > 50 and h < 20 and aspect_ratio > 4.0):  # 1.5, 1.8, 2.0  area / perimeter < 2.0 이게 면적조건
                cv2.drawContours(gray, [c], -1, 0, -1)
            # if area / perimeter < 2.0:
            #     print("얇은 선 제거됨:", area, perimeter)
            #     cv2.drawContours(gray, [c], -1, 0, -1)
            #
            # elif w > 50 and h < 20 and aspect_ratio > 3.0:
            #     print("가로로 두꺼운 선 제거됨:", w, h, aspect_ratio)
            #     cv2.drawContours(gray, [c], -1, 0, -1)

        binary = gray
        #cv2.imshow("gray", gray)



        left_base, right_base = self.plothistogram(binary)

        # draw_info, windows = self.edge_based_lane_detect(bev, show_debug=True)
        draw_info, windows = self.slide_window_search(binary, left_base, right_base)
        bottom_y = self.warped_image_height - 1
        # 검출 여부: fit이 이번 프레임에서 갱신되었는지를 기준으로
        self.left_detected = len(draw_info['left_fitx']) > 30 and not np.allclose(draw_info['left_fit'], [0, 0])
        self.right_detected = len(draw_info['right_fitx']) > 30 and not np.allclose(draw_info['right_fit'], [0, 0])

        # 양쪽 차선이 모두 인식된 경우
        if self.left_detected and self.right_detected:
            self.fallback_active = False
            left_x = draw_info['left_fit'][0] * bottom_y + draw_info['left_fit'][1]
            right_x = draw_info['right_fit'][0] * bottom_y + draw_info['right_fit'][1]
            lane_center = (left_x + right_x) / 2.0
            cte = lane_center - self.warped_center_x
            heading = (draw_info['left_fit'][0] + draw_info['right_fit'][0]) / 2.0
        else:
            if self.left_detected:
                self.fallback_active = False
                left_x = draw_info['left_fit'][0] * bottom_y + draw_info['left_fit'][1]
                lane_center = left_x + 100 if left_x < self.warped_center_x else left_x - 100
                cte = lane_center - self.warped_center_x
                heading = draw_info['left_fit'][0]
            elif self.right_detected:
                self.fallback_active = False
                right_x = draw_info['right_fit'][0] * bottom_y + draw_info['right_fit'][1]
                lane_center = right_x - 100 if right_x > self.warped_center_x else right_x + 100
                cte = lane_center - self.warped_center_x
                heading = draw_info['right_fit'][0]
            else:
                # fallback 모드: 아무 것도 없을 때
                self.fallback_active = True
                cte = 0.0
                heading = 0.0

        # 디버깅용 표시 (필요 시 주석 처리)
        cv2.imshow("lowviewcam", frame2)
        cv2.imshow("BEV", bev)
        cv2.imshow("White Filter", filtered)
        cv2.imshow("Binary", binary)
        cv2.imshow("Windows", windows)
        cv2.waitKey(1)

        print(f"[DEBUG] CTE (px): {cte:.2f}, Heading (slope): {heading:.4f}")
        return draw_info, cte, heading, self.fallback_active, self.left_detected, self.right_detected
