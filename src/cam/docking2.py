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

        # self.detecting_color = rospy.get_param("detecting_color")
        # self.detecting_shape = rospy.get_param("detecting_shape")
        self.detecting_color = 1
        self.detecting_shape = 3
        
        #sub
        self.count_sub = rospy.Subscriber("/count", UInt16, self.count_callback, queue_size=10)
        #pub
        self.cam_data_pub = rospy.Publisher("/cam_data", Cam, queue_size=10)

        self.cam_control_angle = 0
        self.cam_u_thruster = 0
        self.cam_end = False

        # create window for trackbars
        cv.namedWindow('controller')

        # create trackbars for color change

        cv.createTrackbar('lowH', 'controller', 0, 179, self.nothing)
        cv.createTrackbar('highH', 'controller', 255, 179, self.nothing)

        cv.createTrackbar('lowS', 'controller', 0, 255, self.nothing)
        cv.createTrackbar('highS', 'controller', 255, 255, self.nothing)

        cv.createTrackbar('lowV', 'controller', 0, 255, self.nothing)
        cv.createTrackbar('highV', 'controller', 255, 255, self.nothing)

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
        x
        
    def control(self):
        _, cam = self.webcam.read() 

        # 1. 영상 이미지 전처리
        hsv_image = markDetection.image_preprocessing(cam)

        # 2. 탐지 색상 범위에 따라 마스크 형성
        # get current positions of the trackbars
        lower_color = np.array([cv.getTrackbarPos('lowH', 'controller'), cv.getTrackbarPos('lowS', 'controller'), cv.getTrackbarPos('lowV', 'controller')])
        upper_color = np.array([cv.getTrackbarPos('highH', 'controller'), cv.getTrackbarPos('highS', 'controller'), cv.getTrackbarPos('highV', 'controller')])
        mask = cv.inRange(hsv_image, lower_color, upper_color)
        # mask = markDetection.color_filtering(self.detecting_color, hsv_image)

        # 3. 형성된 마스크에서 외곽선 검출
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = np.array(contours)
        min_area = 5000
        contour_info, cam = markDetection.shape_and_label(self.detecting_shape, cam, contours, min_area)
        cv.imshow("CONTROLLER", cam)
        # cv.imshow("MASK", mask)
        control_angle, thruster_value, size = markDetection.move_with_largest(contour_info, cam.shape[1])

        return control_angle, thruster_value, size 

def main():
    docking = Docking()
    rospy.init_node('Docking')
    
    if not docking.webcam.isOpened():
        print("Could not open webcam")
        exit()
            
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        cam_control_angle, cam_u_thruster, size = docking.control()
        docking.publish_value(cam_control_angle, cam_u_thruster, docking.cam_end)
        if (cv.waitKey(1) & 0xFF == 27) or size == 1000:
            docking.cam_end = True
            docking.publish_value(cam_control_angle, cam_u_thruster, docking.cam_end)
            break
        rate.sleep()        

if __name__=="__main__":
    main()
