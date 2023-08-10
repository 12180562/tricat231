#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import UInt16
import cam.markDetection as markDetection

class Docking :
    def __init__(self):
        # 웹캠 설정
        self.cap = cv.VideoCapture(0)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

        # 값 초기화
        self.detecting_color = 0
        self.detecting_shape = 0
        self.target_detect_area = 10
        self.max_area = 0
        self.docking_point = 0

        self.color_bounds = {
            "Blue": ([120, 50, 50], [140, 255, 255]),
            "Green": ([40, 50, 50], [80, 255, 255]),
            # "Red": ([0, 50, 50], [10, 255, 255]),
            "Red": ([115, 183, 176], [130, 255, 255]), 
            "Orange": ([10, 50, 50], [30, 255, 255]),
            "Black": ([0, 0, 0], [180, 255, 50])  # Value is less than 50 to indicate blackness
        }
        self.shape = {
            "0": "Circle",
            "3": "Triangle",
            "4": "Rectangle",
            "12": "Cross"
        }

def main():
    docking = Docking()
    # time_trigger = markDetection.TimeBasedTrigger(3)
    rospy.init_node('Docking')
    
    if not docking.cap.isOpened(): # 캠이 연결되지 않았을 경우 # true시 캠이 잘 연결되어있음
        print("Could not open webcam")
        exit()
            
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        # 프레임 읽기
        _, img = docking.cap.read()

        # ROI 선택
        roi, color_roi = markDetection.image_preprocessing(img)

        # 평균 색상 계산 & HSV로 변환
        avg_color = markDetection.get_average_color_rgb(color_roi)
        hsv_color = markDetection.rgb_to_hsv(avg_color[0], avg_color[1], avg_color[2])

        # 변환된 HSV가 포함된 범위를 반환
        color, color_bounds = markDetection.get_color_bounds(hsv_color, docking.color_bounds)
        if color is None:
            docking.detecting_color = "Black"
        else:
            docking.detecting_color = color

        # HSV 이미지 생성
        hsv_image = cv.cvtColor(img, cv.COLOR_RGB2HSV)

        # 색상 마스크 생성
        if color is not None:
            color_mask = markDetection.apply_color_mask(hsv_image, color_bounds)
        else:
            color_mask = markDetection.apply_color_mask(hsv_image, ([0, 0, 0], [180, 255, 50]))
        
        cv.imshow('color_mask', color_mask)

        # 컨투어 찾기
        contours, _ = cv.findContours(color_mask, cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            pass
        else:
            largest_contour = max(contours, key = cv.contourArea)
            center = markDetection.find_centroid(largest_contour)
            approx = cv.approxPolyDP(largest_contour, cv.arcLength(largest_contour, True) * 0.02, True)
            area = cv.contourArea(approx)
            line_num = len(approx)
        
            if line_num == 3:
                shape = "triangle"
            elif line_num == 4:
                shape = "rectangle"
            elif line_num == 12:
                shape = "cross"
            else:
                _, radius = cv.minEnclosingCircle(approx)  # 원으로 근사
                ratio = radius * radius * 3.14 / (area + 0.000001)  # 해당 넓이와 정원 간의 넓이 비
                if 0.5 < ratio < 2:  # 원에 가까울 때만 필터링
                    shape = "circle"
                else:
                    shape = "unknown"
            cv.circle(img, center, 5, (255, 0, 0), -1)
            markDetection.setLabel(img, largest_contour, shape)
        
        # 도킹 포인트로 들어가기
        if color is not None:
            if color == "Red":
                if shape == "cross":
                    docking.docking_point = 1
                elif shape == "triangle":
                    docking.docking_point = 2
                else:
                    pass
            elif color == "Green":
                docking.docking_point = 1
            elif color == "Orange":
                if shape == "cross" or shape == "circle":
                    docking.docking_point = 2
                elif shape == "triangle":
                    docking.docking_point = 3
                else:
                    pass
            elif color == "Black":
                docking.docking_point = 3
        else:
            pass
        
        print(docking.docking_point)

        cv.imshow('frame', img)
        # print(hsv_color)
        cv.imshow('roi', color_roi)
        
        if cv.waitKey(1) == 27:
            docking.cap.release()
            cv.destroyAllWindows()
            break
        
        rate.sleep()
    docking.cap.release()
    cv.destroyAllWindows()
          

if __name__=="__main__":
    main()