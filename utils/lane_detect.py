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
        self.source = np.float32([[100, 177], [0, 319], [495, 177], [612, 319]])
        self.destination = np.float32([[0, 0], [0, 520], [440, 0], [440, 520]])
        self.transform_matrix = cv2.getPerspectiveTransform(self.source, self.destination)

        # BEV 이미지 중심 좌표
        self.warped_center_x = 230
        self.warped_image_height = 520

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

    def color_filter(self, image):
        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

        # 튜닝된 흰색 범위: H 전체, L은 160 이상, S는 0~120 정도 허용
        lower_white = np.array([0, 170, 0])
        upper_white = np.array([180, 255, 60])

        white_mask = cv2.inRange(hls, lower_white, upper_white)
        masked = cv2.bitwise_and(image, image, mask=white_mask)
        return masked

    def sliding_window_search(self, binary, left_base, right_base):
        nwindows = 30
        window_height = np.int_(binary.shape[0] / nwindows)
        nonzero = binary.nonzero()
        nonzeroy, nonzerox = nonzero[0], nonzero[1]
        margin, minpix = 80, 30
        left_current, right_current = left_base, right_base
        left_lane_inds, right_lane_inds = [], []

        # 디버깅용 출력 이미지 생성 (3채널 컬러)
        out_img = np.dstack((binary, binary, binary)) * 255

        for window in range(nwindows):
            win_y_low = binary.shape[0] - (window + 1) * window_height
            win_y_high = binary.shape[0] - window * window_height
            win_xleft_low = left_current - margin
            win_xleft_high = left_current + margin
            win_xright_low = right_current - margin
            win_xright_high = right_current + margin

            # 사각형 시각화
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

        left_fit = np.polyfit(lefty, leftx, 1) if len(leftx) > 0 else [0, 0]
        right_fit = np.polyfit(righty, rightx, 1) if len(rightx) > 0 else [0, 0]

        # 피팅된 선 그리기
        ploty = np.linspace(0, binary.shape[0] - 1, binary.shape[0])
        left_fitx = left_fit[0] * ploty + left_fit[1]
        right_fitx = right_fit[0] * ploty + right_fit[1]

        for i in range(len(ploty)):
            cv2.circle(out_img, (int(left_fitx[i]), int(ploty[i])), 1, (255, 0, 0), -1)
            cv2.circle(out_img, (int(right_fitx[i]), int(ploty[i])), 1, (0, 0, 255), -1)

        return left_fit, right_fit, out_img

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

    def compute_lane_control(self):
        ret2, frame2 = self.read_downward_camera()
        if not ret2:
            return None

        bev = self.warpping(frame2)
        filtered = self.color_filter(bev)
        gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        histogram = np.sum(binary[binary.shape[0]//2:, :], axis=0)
        midpoint = len(histogram) // 2
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        left_fit, right_fit, windows = self.sliding_window_search(binary, left_base, right_base)
        bottom_y = self.warped_image_height - 1
        left_x = left_fit[0] * bottom_y + left_fit[1]
        right_x = right_fit[0] * bottom_y + right_fit[1]

        lane_center = (left_x + right_x) / 2.0
        cte = lane_center - self.warped_center_x
        heading = (right_x - left_x) / (right_x + left_x + 1e-5)

        # 디버깅용 표시 (필요 시 주석 처리)
        cv2.imshow("lowviewcam", frame2)
        cv2.imshow("BEV", bev)
        cv2.imshow("White Filter", filtered)
        cv2.imshow("Binary", binary)
        cv2.imshow("Windows", windows)
        cv2.waitKey(1)

        return {
            'cte': cte,
            'heading_error': heading,
            'fallback': False
        }
