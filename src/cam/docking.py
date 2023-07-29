#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import UInt16
from tricat231_pkg.msg import Cam
import markDetection_0721

class Docking :
    def __init__(self):
        self.end = False
        self.webcam = cv.VideoCapture(0)
        # if self.end:
        #     self.webcam = cv.VideoCapture(0)
        self.detecting_color = rospy.get_param("detecting_color")
        self.detecting_shape = rospy.get_param("detecting_shape")

        #sub
        self.count_sub = rospy.Subscriber("/count", UInt16, self.count_callback, queue_size=10)
        #pub
        self.cam_data_pub = rospy.Publisher("/cam_data", Cam, queue_size=10)

        self.cam_control_angle = 0
        self.cam_u_thruster = 0
        self.cam_end = False

    def publish_value(self, cam_control_angle, u_thruster, cam_end) :
        self.cam_control_angle = cam_control_angle
        self.cam_u_thruster = u_thruster
        self.cam_end = cam_end
        self.cam_data_pub.publish(self.cam_control_angle,  self.cam_u_thruster, self.cam_end)
    
    def count_callback(self, msg):
        self.end = msg.data
        
    def run(self):
        # webcam = cv.VideoCapture(0)
        # if not self.webcam.isOpened(): # 캠이 연결되지 않았을 경우 # true시 캠이 잘 연결되어있음
        #     print("Could not open webcam")
        #     exit()

        # while self.webcam.isOpened(): 
        _, cam = self.webcam.read() # webcam으로 연결된 정보 읽어오기
            # cv.imshow('webcam', cam) # webcam 창에 cam 보이기
            # raw_img = cv.imread(webcam, cv.IMREAD_COLOR)
            # cv.imshow("RAW_IMG", raw_img)

        # 1. 영상 이미지 전처리
        hsv_image = markDetection_0721.image_preprocessing(cam)

        # 2. 탐지 색상 범위에 따라 마스크 형성
        mask = markDetection_0721.color_filtering(self.detecting_color, hsv_image)

        # 3. 형성된 마스크에서 외곽선 검출 ( datatype 변경 )
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) # 컨투어 검출
        contours = np.array(contours)
        min_area = 5000
        #    contours = contours.astype(np.float)
            
        # contour_info, raw_image = markDetection.shape_and_label(self.detecting_shape, raw_image, contours)
        contour_info, raw_image = markDetection_0721.shape_and_label(self.detecting_shape, cam, contours, min_area)
        cv.imshow("CONTROLLER", raw_image)
        # h,w,c = raw_image.shape # 원본 이미지에서 가로 길이 받아오기
        # move_with_largest(contour_info, w)
        control_angle, thruster_value, size = markDetection_0721.move_with_largest(contour_info, raw_image.shape[1])

        return control_angle, thruster_value, size 

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
    docking = Docking()
    rospy.init_node('Docking')
    
    if not docking.webcam.isOpened(): # 캠이 연결되지 않았을 경우 # true시 캠이 잘 연결되어있음
            print("Could not open webcam")
            exit()
            
    while docking.webcam.isOpened():
        cam_control_angle, u_thruster, size = docking.run()
        if (cv.waitKey(1) & 0xFF == 27) or size == 100: # esc버튼 누르면 창 꺼짐
            docking.finish = True
            break
            
    rate = rospy.Rate(10) # 10Hz
    
    try:
        while not rospy.is_shutdown():
            docking.publish_value(cam_control_angle, u_thruster, docking.cam_end)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()