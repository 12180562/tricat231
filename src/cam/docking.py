#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
import cv2 as cv
from std_msgs.msg import UInt16
from tricat231_pkg.msg import Cam
import cam.markDetection as markDetection
class Docking :
    def __init__(self):
        self.end = False
        self.webcam = cv.VideoCapture(0)

        self.detecting_color = rospy.get_param("detecting_color")
        self.detecting_shape = rospy.get_param("detecting_shape")
        # self.color_bounds = rospy.get_param("color_bounds")
        self.min_area = rospy.get_param("min_area")
        self.target_detect_area = rospy.get_param("target_detect_area")
        # self.detecting_color = 3
        # self.detecting_shape = 3
        self.color_bounds = {
            1: ([89, 50, 50], [138, 255, 255]),  # Blue
            2: ([30, 50, 50], [80, 255, 255]),   # Green
            3: ([0, 116, 153], [255, 255, 255]),  # Red
            4: ([10, 200, 213], [23, 255, 255]),  # Orange
            5: ([96, 60, 27], [144, 255, 255])   # Black
        }
        # self.min_area = 10
        # self.target_detect_area = 20
        
        self.cam_control_angle = 0
        self.cam_u_thruster = 0
        self.cam_end = False
        self.cam_detect = False

        #sub
        self.count_sub = rospy.Subscriber("/count", UInt16, self.count_callback, queue_size=10)
        #pub
        self.cam_data_pub = rospy.Publisher("/cam_data", Cam, queue_size=10)

    def count_callback(self, msg):
        self.end = msg.data

    def publish_value(self, cam_control_angle, u_thruster, cam_end, detect) :
        self.cam_control_angle = cam_control_angle
        self.cam_u_thruster = u_thruster
        self.cam_end = cam_end
        self.cam_detect = detect

        cam_msg = Cam()
        cam_msg.cam_control_angle.data = self.cam_control_angle
        cam_msg.cam_u_thruster.data = self.cam_u_thruster
        cam_msg.cam_end.data = self.cam_end
        cam_msg.cam_detect.data = self.cam_detect
        self.cam_data_pub.publish(cam_msg)
        
    def control(self):
        _, cam = self.webcam.read() # webcam으로 연결된 정보 읽어오기

        # 영상 이미지 전처리
        raw_image = cam
        hsv_image = markDetection.image_preprocessing(raw_image, brightness=False, blur=False)

        # 탐지 색상 범위에 따라 마스크 형성
        mask = markDetection.color_filtering(self.detecting_color, self.color_bounds, hsv_image)

        # 형성된 마스크에서 외곽선 검출
        # contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # 모폴로지 연산(빈 공간 완화 & 노이즈 제거) 사용 시
        contours = markDetection.morph_contour(mask)

        contour_info = markDetection.shape_detection(self.detecting_shape, self.target_detect_area, self.min_area, contours)
        # img = markDetection.window(raw_image, contour_info, "Circle")
        # cv.imshow("CONTROLLER", img)
        cv.imshow("MASK", mask)

        control_angle, thruster_value, size, detect  = markDetection.move_to_largest(contour_info, raw_image.shape[1])
        print(control_angle, thruster_value, size, detect)

        return control_angle, thruster_value, size, detect

def main():
    docking = Docking()
    time_trigger = markDetection.TimeBasedTrigger(3)
    rospy.init_node('Docking')
    
    if not docking.webcam.isOpened(): # 캠이 연결되지 않았을 경우 # true시 캠이 잘 연결되어있음
        print("Could not open webcam")
        exit()
            
    rate = rospy.Rate(10) # 10Hz
    # while docking.webcam.isOpened():
    while not rospy.is_shutdown():
        cam_control_angle, cam_u_thruster, size, detect = docking.control()
        if time_trigger.trigger(detect):
            docking.publish_value(cam_control_angle, cam_u_thruster, docking.cam_end, detect)
        else:
            docking.publish_value(cam_control_angle, cam_u_thruster, docking.cam_end, detect=False)

        if (cv.waitKey(1) & 0xFF == 27) or size == 100: # 카메라 창에서 esc버튼 꺼짐
            docking.cam_end = True
            docking.publish_value(cam_control_angle, cam_u_thruster, docking.cam_end, detect)
            break
        
        print(docking.cam_end, docking.cam_detect)
        rate.sleep()        

if __name__=="__main__":
    main()