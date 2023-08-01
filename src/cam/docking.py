#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import UInt16
from tricat231_pkg.msg import Cam
import cam.markDetection as markDetection

class Docking :
    def __init__(self):
        self.end = False
        self.webcam = cv.VideoCapture(0)
        self.init_lower_color = np.array([0, 0, 0])
        self.init_upper_color = np.array([255, 255, 255])

        # self.detecting_color = rospy.get_param("detecting_color")
        # self.detecting_shape = rospy.get_param("detecting_shape")
        self.detecting_color = 1
        self.detecting_shape = 0
        
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

        cam_msg = Cam()
        cam_msg.cam_control_angle.data = self.cam_control_angle
        cam_msg.cam_u_thruster.data = self.cam_u_thruster
        cam_msg.cam_end.data = self.cam_end
        self.cam_data_pub.publish(cam_msg)

    def count_callback(self, msg):
        self.end = msg.data

    # callback function for trackbar, does nothing
    def nothing(self, x):
        pass
        
    def control(self):
        _, cam = self.webcam.read() # webcam으로 연결된 정보 읽어오기
        # cv.imshow('webcam', cam) # webcam 창에 cam 보이기
        # raw_img = cv.imread(webcam, cv.IMREAD_COLOR)
        # cv.imshow("RAW_IMG", raw_img)

        # 1. 영상 이미지 전처리
        hsv_image = markDetection.image_preprocessing(cam)

        # 2. 탐지 색상 범위에 따라 마스크 형성
        mask = markDetection.color_filtering(self.detecting_color, hsv_image)

        # 3. 형성된 마스크에서 외곽선 검출 ( datatype 변경 )
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) # 컨투어 검출
        contours = np.array(contours)
        min_area = 5000
        #    contours = contours.astype(np.float)
            
        # contour_info, raw_image = markDetection.shape_and_label(self.detecting_shape, raw_image, contours)
        contour_info, raw_image = markDetection.shape_and_label(self.detecting_shape, cam, contours, min_area)
        # contour_info, mask = markDetection.shape_and_label(self.detecting_shape, cam, contours, min_area)
        cv.imshow("CONTROLLER", raw_image)
        # cv.imshow("MASK", mask)
        # h,w,c = raw_image.shape # 원본 이미지에서 가로 길이 받아오기
        # move_with_largest(contour_info, w)
        control_angle, thruster_value, size = markDetection.move_with_largest(contour_info, raw_image.shape[1])

        return control_angle, thruster_value, size 

    # def get_trackbar_pos(self):
    #     """get trackbar poses and set each values"""
    #     self.color_range[0][0] = cv.getTrackbarPos("color1 min", "controller")
    #     self.color_range[1][0] = cv.getTrackbarPos("color1 max", "controller")
    #     self.color_range[0][1] = cv.getTrackbarPos("color2 min", "controller")
    #     self.color_range[1][1] = cv.getTrackbarPos("color2 max", "controller")
    #     self.color_range[0][2] = cv.getTrackbarPos("color3 min", "controller")
    #     self.color_range[1][2] = cv.getTrackbarPos("color3 max", "controller")
    #     self.mark_detect_area = cv.getTrackbarPos("mark_detect_area", "controller")
    #     self.target_detect_area = cv.getTrackbarPos("target_detect_area", "controller")
    #     self.arrival_target_area = cv.getTrackbarPos("arrival_target_area", "controller")

def main():
    docking = Docking()
    rospy.init_node('Docking')
    
    if not docking.webcam.isOpened(): # 캠이 연결되지 않았을 경우 # true시 캠이 잘 연결되어있음
        print("Could not open webcam")
        exit()
            
    rate = rospy.Rate(10) # 10Hz
    # while docking.webcam.isOpened():
    while not rospy.is_shutdown():
         # get current positions of the trackbars
        cv.namedWindow('Colorbars')
        lower_color = np.array([cv.getTrackbarPos('lowH', 'Colorbars'), cv.getTrackbarPos('lowS', 'Colorbars'), cv.getTrackbarPos('lowV', 'Colorbars')])
        upper_color = np.array([cv.getTrackbarPos('highH', 'Colorbars'), cv.getTrackbarPos('highS', 'Colorbars'), cv.getTrackbarPos('highV', 'Colorbars')])


        cam_control_angle, cam_u_thruster, size = docking.control()
        docking.publish_value(cam_control_angle, cam_u_thruster, docking.cam_end)
        if (cv.waitKey(1) & 0xFF == 27) or size == 100: # 카메라 창에서 esc버튼 꺼짐
            docking.cam_end = True
            docking.publish_value(cam_control_angle, cam_u_thruster, docking.cam_end)
            break
        rate.sleep()        

if __name__=="__main__":
    main()