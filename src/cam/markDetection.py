#!/usr/bin/env python
#-*- coding:utf-8 -*-

import time
import cv2 as cv
import numpy as np

# 평균 밝기로 화면 밝기 조정
def mean_brightness(img):
    """
    Args:
        img (numpy.ndarray): 입력 이미지
    
    Returns:
        dst (numpy.ndarray): 평균 밝기가 조절된 이미지
    """
    fixed = 100  # 이 값 주변으로 평균 밝기 조절함
    m = cv.mean(img)  # 평균 밝기
    scalar = (-int(m[0]) + fixed, -int(m[1]) + fixed, -int(m[2]) + fixed, 0)
    dst = cv.add(img, scalar)
    return dst

def roi_selection(img):
    # 프레임의 크기를 구함
    height, width, _ = img.shape

    # ROI를 프레임의 하단 15%로 설정
    roi_start_w = int(width * 0.15)
    roi_start_h = int(height * 0.25)
    roi_end_w = int(width* 0.85)
    roi_end_h = height

    # ROI 지정
    roi = img[roi_start_h:roi_end_h, roi_start_w:roi_end_w]
    return roi

def color_roi_selection(img):
    # 프레임의 크기를 구함
    height, width, _ = img.shape
    region = 50
    # ROI를 프레임의 중앙 50x50으로 설정
    roi_start_w = width // 2 - region // 2
    roi_start_h = height // 2 - region // 2
    roi_end_w = width // 2 + region // 2
    roi_end_h = height // 2 + region // 2

    # ROI 지정
    center_roi = img[roi_start_h:roi_end_h, roi_start_w:roi_end_w]
    return center_roi

# 영상 이미지 전처리
def image_preprocessing(raw_image, brightness=False, blur=True):
    """
    Args:
        raw_img (np.ndarray): Input raw image from the camera
        blur (bool): Whether to do Gaussian Blur or not. Fix kernel size with 5
        brightness (bool): Whether to adjust brightness with mean brightness value
    
    Returns:
        np.ndarray: preprocessed image
    """ 
    img = raw_image

    if brightness == True:
        img = mean_brightness(img)
    if blur == True:
        """
        Gaussian blur가 중심 픽셀 주위에 커널을 적용하기 때문에 적용할 시 커널 크기는 항상 홀수여야 함
        홀수 크기의 커널을 사용하면 중심 픽셀이 항상 있지만, 짝수 크기의 커널을 사용하면 중심 픽셀이 없음
        (4, 4): 불가 / (5, 5): 가능
        """
        img = cv.GaussianBlur(img, (5, 5), 0)
    roi = roi_selection(img)
    color_roi = color_roi_selection(roi)
    
    return roi, color_roi

# def image_preprocessing(raw_image, brightness=False, blur=True):
    
#     img = raw_image

#     if brightness == True:
#         img = mean_brightness(img)
#     if blur == True:
#         """
#         Gaussian blur가 중심 픽셀 주위에 커널을 적용하기 때문에 적용할 시 커널 크기는 항상 홀수여야 함
#         홀수 크기의 커널을 사용하면 중심 픽셀이 항상 있지만, 짝수 크기의 커널을 사용하면 중심 픽셀이 없음
#         (4, 4): 불가 / (5, 5): 가능
#         """
#         img = cv.GaussianBlur(img, (5, 5), 0)
#     hsv_image = cv.cvtColor(img, cv.COLOR_RGB2HSV)
    
#     return hsv_image

def rgb_to_hsv(r, g, b):
    return tuple(cv.cvtColor(np.uint8([[[b, g, r]]]), cv.COLOR_BGR2HSV)[0][0])

def get_average_color_rgb(region):
    return np.average(np.average(region, axis=0), axis=0)

# def create_color_mask(hsv_frame, avg_hsv):
#     # 평균 색상 주변의 HSV 범위를 정의
#     lower_color = np.array([avg_hsv[0][0][0] - 10, 100, 100])
#     upper_color = np.array([avg_hsv[0][0][0] + 10, 255, 255])

#     # HSV 범위에 맞는 마스크 생성
#     color_mask = cv.inRange(hsv_frame, lower_color, upper_color)
#     return color_mask

# 이미지 내 특정 색상 검출 후 마스크로 변환
# def get_color_bounds(detecting_color, color_bounds):
#     """
#     Args:
#         detecting_color (int)
#         color_bounds (Dictionary)
    
#     Returns:
#         lower_color (ndarray)
#         upper_color (ndarray)
#     """ 
#     if str(detecting_color) in color_bounds.keys():
#         lower_color = np.array(color_bounds[str(detecting_color)][0])
#         upper_color = np.array(color_bounds[str(detecting_color)][1])
#     else:
#         pass

#     return lower_color, upper_color

def color_filtering(detecting_color, color_bounds, img):
    """
    Args:
        detecting_color (int)
        color_bounds (Dictionary)
        img (np.ndarray): preprocessed image with 3 channels
    
    Returns:
        mask (np.ndarray): gray-scale image with specific color range
    """
    lower_color, upper_color = get_color_bounds(detecting_color, color_bounds)
    mask = cv.inRange(img, lower_color, upper_color) # 색상 범위에 해당하는 마스크 생성
    return mask

def morph_contour(img):
    # 모폴로지 연산
    morph_kernel = np.ones((9, 9), np.uint8)
    morph = cv.morphologyEx(img, cv.MORPH_CLOSE, morph_kernel)
    contours, _ = cv.findContours(morph, cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)
    return contours

# 원하는 도형의 윤곽선 면적과 중심점 찾기, 도형 labelling    
def find_centroid(contour):
    center = (0,0)
    # 윤곽선의 중심점 계산
    M = cv.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        center = (cx, cy)
        return center
    else:
        return None
    
def shape_detection(detecting_shape, target_detect_area, min_area, contours):
    detect = 30 # None
    # max_area = 0
    contour_info = [detect, None, None, None]
    for contour in contours:
        # 도형 근사
        """
        cv2.approxPolyDP(curve, epsilon, closed, approxCurve=None) -> approxCurve : 외곽선을 근사화(단순화)
            • curve: 입력 곡선 좌표. numpy.ndarray. shape=(K, 1, 2)
            • epsilon: 근사화 정밀도 조절. 입력 곡선과 근사화 곡선 간의 최대 거리. e.g) cv2.arcLength(curve) * 0.02
            • closed: True를 전달하면 폐곡선으로 인식
            • approxCurve: 근사화된 곡선 좌표. numpy.ndarray. shape=(K', 1, 2)

        cv2.arcLength(curve, closed) -> retval: 외곽선 길이를 반환
            • curve: 외곽선 좌표. numpy.ndarray. shape=(K, 1, 2)
            • closed: True이면 폐곡선으로 간주
            • retval: 외곽선 길이

        approxPolyDP()의 반환인 approx의 구조 (삼각형일 때)
            • type: numpy.ndarray
            • shape: (3, 1, 2) -> 변 개수, 1, [행, 열] 정보이므로 요소 두 개
            • 예: [[[105, 124]], [[107, 205]], [[226, 163]]]
        때에 따라서는 같은 도형이 여러 개 발견될 수도 있음
        """
        approx = cv.approxPolyDP(contour, cv.arcLength(contour, True) * 0.02, True)

        # 도형 넓이
        area = cv.contourArea(approx)
        # 인식 면적 제한
        if area < min_area: 
            continue
        # 변의 개수
        line_num = len(approx)
        # 탐지 여부 및 도형 구분
        if (line_num == detecting_shape) and (area >= target_detect_area):
            if detecting_shape == 0: # 원일 때
                _, radius = cv.minEnclosingCircle(approx)  # 원으로 근사
                ratio = radius * radius * 3.14 / (area + 0.000001)  # 해당 넓이와 정원 간의 넓이 비
                if 0.5 < ratio < 2:  # 원에 가까울 때만 필터링
                    detect = 10 # True
                    center = find_centroid(contour)
            else:
                detect = 10 # True
                center = find_centroid(contour)
        else:
            detect = 20 # False

        if detect == 10:
            if area > max_area:
                contour_info = [detect, area, center, contour]
                max_area = area
            else:
                detect = 30 # None
                contour_info = [detect, None, None, None]
        else:
            contour_info = [detect, None, None, None]
        
        print(line_num)
        print(area)

    return contour_info

def window(img, contour_info, label): # 중심점이 유효한지 확인
    if len(contour_info) > 0 and contour_info[0] == 10:
        cv.circle(img, contour_info[2], 5, (255, 0, 0), -1)
        setLabel(img, contour_info[3], label)
    else:
        img = img
    return img

def setLabel(img, pts, label):
    (x,y,w,h) = cv.boundingRect(pts)
    pt1 = (x,y)
    pt2 = (x+w, y+h)
    cv.rectangle(img, pt1, pt2, (0,255,0), 2)
    cv.putText(img, label, (pt1[0], pt1[1]-3), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))

def move_to_largest(contour_info, raw_image_width):
    # 제일 큰 도형에 대한 연산 수행
    # Limage_limit = raw_image_width / 2 - 10
    # Rimage_limit = raw_image_width / 2 + 10
    Limage_limit = raw_image_width / 2
    Rimage_limit = raw_image_width / 2
    control_angle = 0
    thruster = 1500
    size = 0
    a = 2 # 도형 크기 비 (직진 여부 확인용)
    detect = 30 # None
    if len(contour_info) > 1:
        detect, largest_area, largest_center, _ = contour_info
        if contour_info[2] is not None and detect == 10:
            centroid_x = largest_center[0]
            largest_width = largest_area  # 도형의 가로 길이를 largest_area로 간주

            if centroid_x < Limage_limit: # Turn Left
                control_angle = 1

            elif centroid_x > Rimage_limit: # Turn Right
                control_angle = 2

            elif Limage_limit < centroid_x < Rimage_limit: # Move Front
                if largest_width < raw_image_width / a :
                    size = 10
                    control_angle = 0
                    thruster = 1550
                elif largest_width > raw_image_width / a: # STOP
                    size = 100
                    control_angle = 0
                    thruster = 1500
    
    return control_angle, thruster, size, detect

def get_color_bounds(hsv_color, color_bounds):
    for color, bounds in color_bounds.items():
        if (bounds[0][0] <= hsv_color[0] <= bounds[1][0]) and (bounds[0][1] <= hsv_color[1] <= bounds[1][1]) and (bounds[0][2] <= hsv_color[2] <= bounds[1][2]):
            return color, bounds
    return None, None


def apply_color_mask(hsv_image, color_bounds):
        if color_bounds is None:
            return None
        lower_bound = np.array(color_bounds[0], dtype=np.uint8)
        upper_bound = np.array(color_bounds[1], dtype=np.uint8)
        color_mask = cv.inRange(hsv_image, lower_bound, upper_bound)
        return color_mask

class TimeBasedTrigger:
    def __init__(self, duration):
        self.start_time = None
        self.duration = duration

    def trigger(self, value):
        if value:
            if self.start_time is None:
                self.start_time = time.time()
            elif time.time() - self.start_time >= self.duration:
                return True
        else:
            self.start_time = None

        return False