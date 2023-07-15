#!/usr/bin/env python
# -*- coding:utf-8 -*-

import cv2 as cv
import numpy as np

import sys, os
import rospy
from std_msgs.msg import Float64, UInt16

import markDetection

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

class Docking :
    def __init__(self):
        
        self.servo_range = rospy.get_param("servo_range")
        self.servo_middle = int((self.servo_range[0] + self.servo_range[1]) / 2) 

        self.u_servo = self.servo_middle
        self.u_thruster = self.thruster_min

        self.servo_pub = rospy.Publisher("/servo",UInt16)
        self.thruster_pub = rospy.Publisher("/thruster",UInt16)
        
    # def cam_callback(self, msg):
    # img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    # if img.size == (640 * 480 * 3):
    #     self.raw_img = img
    # else:
    #     pass

    # def get_trackbar_pos(self):
    #     """get trackbar poses and set each values"""
    #     # self.color_range[0][0] = cv2.getTrackbarPos("color1 min", "controller")
    #     # self.color_range[1][0] = cv2.getTrackbarPos("color1 max", "controller")
    #     # self.color_range[0][1] = cv2.getTrackbarPos("color2 min", "controller")
    #     # self.color_range[1][1] = cv2.getTrackbarPos("color2 max", "controller")
    #     # self.color_range[0][2] = cv2.getTrackbarPos("color3 min", "controller")
    #     # self.color_range[1][2] = cv2.getTrackbarPos("color3 max", "controller")
    #     # self.mark_detect_area = cv2.getTrackbarPos("mark_detect_area", "controller")
    #     # self.target_detect_area = cv2.getTrackbarPos("target_detect_area", "controller")
    #     # self.arrival_target_area = cv2.getTrackbarPos("arrival_target_area", "controller")


def main():
#    rospy.init_node("")

    """
    detecting_color : Blue = 1, Green = 2, Red = 3
    detecting_shape : Circle = 0, Triangle = 3, Rectangle = 4, cross = 12
    """
    detecting_color = 3
    detecting_shape = 3

    webcam = cv.VideoCapture(2) # 캠 연결된 USB 포트 번호 수정하기

    if not webcam.isOpened(): # 캠이 연결되지 않았을 경우 # true시 캠이 잘 연결되어있음
        print("Could not open webcam")
        exit()

    # 카메라에 보이는 이미지가 연결되어 동영상으로 보이는 형태
    while webcam.isOpened(): 
        ret, cam = webcam.read() # webcam으로 연결된 정보 읽어오기
#        cv.imshow('webcam', cam) # webcam 창에 cam 보이기
        # raw_img = cv.imread(webcam, cv.IMREAD_COLOR)
        # cv.imshow("RAW_IMG", raw_img)

    # 1. 영상 이미지 전처리
        raw_image = cam
        img0 = markDetection.mean_brightness(raw_image) # 평균 밝기로 보정하는 함수
        img = cv.GaussianBlur(img0, (5, 5), 0) # 가우시안 필터 적용 # (n,n) : 가우시안 필터의 표준편차. 조정하면서 해야 함
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV) # BGR 형식의 이미지를 HSV 형식으로 전환

    # 2. 탐지 색상 범위에 따라 마스크 형성
        mask = markDetection.color_filtering(detecting_color, hsv_image)

    # 3. 형성된 마스크에서 외곽선 검출 ( datatype 변경 )
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) # 컨투어 검출
        contours = np.array(contours)
    #    contours = contours.astype(np.float)
        
        contour_info, raw_image = markDetection.shape_and_label(detecting_shape, raw_image, contours)
        cv.imshow("CONTROLLER", raw_image)
        markDetection.move_with_largest(contour_info, raw_image.shape[1])

        if cv.waitKey(1) & 0xFF == 27: # esc버튼 누르면 창 꺼짐
            break 

if __name__=="__main__":
    main()