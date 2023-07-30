#!/usr/bin/env python
#-*- coding:utf-8 -*-

import cv2 as cv
import numpy as np

# 영상 이미지 전처리
def image_preprocessing(cam): 
    raw_image = cam
    img0 = mean_brightness(raw_image) # 평균 밝기로 보정하는 함수
    img = cv.GaussianBlur(img0, (5, 5), 0) # 가우시안 필터 적용 # (n,n) : 가우시안 필터의 표준편차. 조정하면서 해야 함
    hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV) # BGR 형식의 이미지를 HSV 형식으로 전환
    return hsv_image

def mean_brightness(img):
    fixed = 100  # 이 값 주변으로 평균 밝기 조절함
    m = cv.mean(img)  # 평균 밝기
    scalar = (-int(m[0]) + fixed, -int(m[1]) + fixed, -int(m[2]) + fixed, 0)
    dst = cv.add(img, scalar)
    return dst

# 탐지 범위에 따른 마스크 형성 및 외곽선 검출  
def show_the_shape_contour(hsv_image,detecting_color):
    mask = color_filtering(detecting_color, hsv_image)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) # 컨투어 검출
    contours = np.array(contours)
#    contours = contours.astype(np.float)
    return contours

# 이미지 내 특정 색상 검출
# def color_filtering(detecting_color, hsv_image):
#     if detecting_color == 1: # Blue
#         lower_color = np.array([110, 50, 50]) # np.array([100, 100, 100])
#         upper_color = np.array([130, 255, 255]) # np.array([130, 255, 255])
#         mask = cv.inRange(hsv_image, lower_color, upper_color) # 색상 범위에 해당하는 마스크 생성
#     elif detecting_color == 2: # Green
#         lower_color = np.array([50, 50, 50]) # np.array([40, 100, 100])
#         upper_color = np.array([70, 255, 255]) # np.array([80, 255, 255])
#         mask = cv.inRange(hsv_image, lower_color, upper_color)
#     elif detecting_color == 3: # Red
#         lower_color = np.array([0, 100, 100]) 
#         upper_color = np.array([10, 255, 255]) 
#         mask = cv.inRange(hsv_image, lower_color, upper_color)
#     elif detecting_color == 4: # Orange
#         lower_color = np.array([10, 100, 100]) #오렌지 알지비 값이 좀 이상할수 있음 알지비기준 255 69 0
#         upper_color = np.array([25, 255, 255])
#         mask = cv.inRange(hsv_image, lower_color, upper_color)
#     elif detecting_color == 5: # Black
#         lower_color = np.array([0, 0, 0])
#         upper_color = np.array([255, 255, 30]) 
#         mask = cv.inRange(hsv_image, lower_color, upper_color)
#     else:
#         pass
#     return mask
def color_filtering(detecting_color, hsv_image):
    if detecting_color == 1: # Blue
        lower_color = np.array([180, 0, 0]) # np.array([100, 100, 100])
        upper_color = np.array([260, 100, 100]) # np.array([130, 255, 255])
        mask = cv.inRange(hsv_image, lower_color, upper_color) # 색상 범위에 해당하는 마스크 생성
    elif detecting_color == 2: # Green
        lower_color = np.array([0, 180, 0]) # np.array([40, 100, 100])
        upper_color = np.array([100, 260, 100]) # np.array([80, 255, 255])
        mask = cv.inRange(hsv_image, lower_color, upper_color)
    elif detecting_color == 3: # Red
        lower_color = np.array([0, 0, 255]) 
        upper_color = np.array([90, 80, 255]) 
        mask = cv.inRange(hsv_image, lower_color, upper_color)
    elif detecting_color == 4: # Orange
        lower_color = np.array([0, 100, 240]) #오렌지 알지비 값이 좀 이상할수 있음 알지비기준 255 69 0
        upper_color = np.array([100, 170, 260])
        mask = cv.inRange(hsv_image, lower_color, upper_color)
    elif detecting_color == 5: # Black
        lower_color = np.array([0, 0, 0])
        upper_color = np.array([60, 70, 100]) 
        mask = cv.inRange(hsv_image, lower_color, upper_color)
    else:
        pass
    return mask

def show_the_shape_info(raw_image, detecting_shape,contours) :
    contour_info, raw_image = shape_and_label(detecting_shape, raw_image, contours)
#    cv.imshow("CONTROLLER", raw_image)
    control_angle_angle, thruster_value = move_with_largest(contour_info, raw_image.shape[1]) # return : control_angle, thruster
    return raw_image, control_angle_angle, thruster_value

###################################
# JJU_0721수정_3 : 함수 이름 find_centroid로 수정했음
# JJU_ : docking.py에서 함수 이름 바꿔주시고, shape_and_label이랑 find_centroid return type 확인해서 수정해야 합니당
###################################

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

###################################
# JJU_0721수정_2 : 원 인식 부분 수정함 & area 변수 계산 추가함 & shape_and_label 함수 매개변수 min_area 추가함
# JJU_ : 지난번에 수정 안된걸 드렸나봐요 (?). 원 인식을 위해서 원래 뒤에 find_area_centroid 함수에서 계산했던 'area' 연산을 이 함수로 뺐어용
###################################

def shape_and_label(detecting_shape, raw_image, contours, min_area):    # 원하는 도형의 윤곽선 면적과 중심점 찾기, 도형 labelling
    contour_info = []
    for contour in contours:
        # 윤곽선의 면적 계산
        area = cv.contourArea(contour)
        if area < min_area: # 인식 면적 제한 두기
            continue
        approx = cv.approxPolyDP(contour, cv.arcLength(contour, True) * 0.5, True)

        # cv2.approxPolyDP(curve, epsilon, closed, approxCurve=None) -> approxCurve : 외곽선을 근사화(단순화)
        #   • curve: 입력 곡선 좌표. numpy.ndarray. shape=(K, 1, 2)
        #   • epsilon: 근사화 정밀도 조절. 입력 곡선과 근사화 곡선 간의 최대 거리. e.g) cv2.arcLength(curve) * 0.02
        #   • closed: True를 전달하면 폐곡선으로 인식
        #   • approxCurve: 근사화된 곡선 좌표. numpy.ndarray. shape=(K', 1, 2)

        # cv2.arcLength(curve, closed) -> retval: 외곽선 길이를 반환
        #   • curve: 외곽선 좌표. numpy.ndarray. shape=(K, 1, 2)
        #   • closed: True이면 폐곡선으로 간주
        #   • retval: 외곽선 길이 

        line_num = len(approx)                
        if detecting_shape == 3 and line_num == 3:  # 삼각형
            center = find_centroid(contour)
            setLabel(raw_image, contour, 'TRIANGLE')
        elif detecting_shape == 4 and line_num == 4:  # 사각형
            center = find_centroid(contour)
            setLabel(raw_image, contour, 'RECTANGLE')
        elif detecting_shape == 12 and line_num == 12:  # 십자가
            center = find_centroid(contour)
            setLabel(raw_image, contour, 'CROSS')
        elif detecting_shape == 0 and line_num == 0: # 원
            _, radius = cv.minEnclosingCircle(approx)  # 원으로 근사
            ratio = radius * radius * 3.14 / (area + 0.000001)  # 해당 넓이와 정원 간의 넓이 비
            if 0.5 < ratio < 2:  # 원에 가까울 때만 필터링
                center = find_centroid(contour)
                setLabel(raw_image, contour, 'CIRCLE')
            else:
                center = None
        else:
            center = None
            
        if area is not None and center is not None:
            contour_info.append((area, center))
            cv.circle(raw_image, center, 5, (255, 0, 0), -1)

    return contour_info, raw_image

def setLabel(img, pts, label):
    (x,y,w,h) = cv.boundingRect(pts)
    pt1 = (x,y)
    pt2 = (x+w, y+h)
    cv.rectangle(img, pt1, pt2, (0,255,0), 2)
    cv.putText(img, label, (pt1[0], pt1[1]-3), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))

def move_with_largest(contour_info, raw_image_width):
    # 제일 큰 도형 선택
#    print("Contour Info Before Filtering:", contour_info)  # Print contour_info before filtering
    contour_info = [info for info in contour_info if info[0] is not None]  # (area, center) 가 None인 경우 필터링
#    print("Contour Info After Filtering:", contour_info)  # Print contour_info after filtering
    Limage_limit = raw_image_width / 2 - 10
    Rimage_limit = raw_image_width / 2 + 10
    control_angle = 93
    thruster = 1550 # thruster_mid
    size = 0
    if len(contour_info) > 0: # contour에 area, center가 입력되었을 때 ( 도형이 1개 이상 인식되었을 때 )
        contour_info.sort(key=lambda x: x[0], reverse=True)  # 도형 면적 기준으로 area, center 내림차순 정렬
        largest_contour = contour_info[0] # 제일 큰 도형 선택
        # 제일 큰 도형에 대한 연산 수행
        largest_area, largest_center = largest_contour
        centroid_x = largest_center[0]
        a = 5 # 도형 크기 비 (직진 여부 확인용)
        largest_width = largest_area  # 도형의 가로 길이를 largest_area로 간주

        if centroid_x < Limage_limit :
        # and largest_width < raw_image_width / a: # center의 x좌표가 화면 절반보다 왼쪽에 있을 때 : 왼쪽으로 회전
        #    print(centroid_x, Limage_limit, largest_width, raw_image_width, "Move Left")
        #    print(contour_info)
            # control_angle
            print("Left")
            control_angle = 81

        elif centroid_x > Rimage_limit :
        # and largest_width < raw_image_width / a: # center의 x좌표가 화면 절반보다 오른쪽에 있을 때 : 오른쪽으로 회전
        #    print(centroid_x, Limage_limit, largest_width, raw_image_width, "Move Right")
        #    print(contour_info)
            print("Right")
            control_angle = 105

        elif Limage_limit < centroid_x < Rimage_limit :
            if largest_width < raw_image_width / a :
                print("Move Front")
                size = 10
                control_angle = 93
                thruster = 1550 # thruster_max
            elif largest_width > raw_image_width / a :
                print("STOP")
                size = 100
                control_angle = 93
                thruster = 1500 # thruster_min
        ## 예외case
        # elif centroid_x < Limage_limit and largest_width > raw_image_width / a : 
        #     print("case1")
        # elif centroid_x > Rimage_limit and largest_width > raw_image_width / a :
        #     print("case2") 
    else:
        print("No contour found")
    print("control_angle : ", control_angle, "thruster : ", thruster)
    return control_angle, thruster, size
#    print(contour_info)  # Print contour_info for debugging


