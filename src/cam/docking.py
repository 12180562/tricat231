#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import math
import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import Float64, UInt16, Bool
import markDetection

class Docking :
    def __init__(self):
        self.end = False
        self.webcam = cv.VideoCapture(0)
        # if self.end:
        #     self.webcam = cv.VideoCapture(0)
         
        """
        detecting_color : Blue = 1, Green = 2, Red = 3, Orange = 4, Black = 5
        detecting_shape : Circle = 0, Triangle = 3, Rectangle = 4, cross = 12
        """
        self.detecting_color = rospy.get_param("detecting_color")
        self.detecting_shape = rospy.get_param("detecting_shape")

        self.u_servo = 93
        self.u_thruster = 1500

        #sub
        self.end_sub = rospy.Subscriber("/end_check", Bool, self.end_callback, queue_size=10)
        #pub
        self.servo_pub = rospy.Publisher("/u_servo_cam" ,UInt16, queue_size= 1)
        self.thruster_pub = rospy.Publisher("/u_thruster_cam",UInt16, queue_size= 1)
        self.finish_pub = rospy.Publisher("/finish_check", Bool, queue_size=10)
        self.finish = False

        self.control_angle = 0
        self.errSum = 0
        self.kp_servo = rospy.get_param("kp_servo")
        self.servo_range = rospy.get_param("servo_range")
        self.servo_middle = int((self.servo_range[0] + self.servo_range[1]) / 2) 

        self.u_thruster = int(rospy.get_param("thruster"))

    def publish_value(self, u_servo, u_thruster) :
        self.u_servo = u_servo
        self.u_thruster = u_thruster
        # 터미널 출력
        # rospy.loginfo("servo_angle = %d", self.u_servo)
        # rospy.loginfo("thruster = %d", self.u_thruster)
        cnt = 0
        if self.end:
            cnt += 1
        if cnt == 1:
            self.servo_pub.publish(int(self.u_servo))
            self.thruster_pub.publish(int(self.u_thruster))
        else:
            pass
    
    def end_callback(self, msg):
        self.end = msg.data
        
    def finish_callback(self, msg):
        self.end = msg.data
        
    def run(self):
        # webcam = cv.VideoCapture(0) # 캠 연결된 USB 포트 번호 수정하기
        # J: 컴퓨터 내장 웹캠은 0, usb 웹캠은 2 아님???

        # J: 이 부분 total_fuzzy, static에서 센서 check하는 부분으로 뺄 수 도 있을 듯?
        # if not self.webcam.isOpened(): # 캠이 연결되지 않았을 경우 # true시 캠이 잘 연결되어있음
        #     print("Could not open webcam")
        #     exit()

        # 카메라에 보이는 이미지가 연결되어 동영상으로 보이는 형태
        # while self.webcam.isOpened(): 
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
            contour_info, raw_image = markDetection.shape_and_label(self.detecting_shape, raw_image, contours, min_area)
            cv.imshow("CONTROLLER", raw_image)
            # h,w,c = raw_image.shape # 원본 이미지에서 가로 길이 받아오기
            # move_with_largest(contour_info, w)
            self.control_angle, thruster_value, size = markDetection.move_with_largest(contour_info, raw_image.shape[1])

            return self.control_angle, thruster_value, size 
    
    def servo_pid_controller(self):
        # start = time.time()
        # generate = self.make_detecting_vector()
        # cross_check = self.delete_vector_inside_obstacle(generate)
        self.run()

        control_angle = self.control_angle
        cp_servo = self.kp_servo * control_angle
        # print(f"error_angle: {self.error_angle}\n")
        # yaw_rate = math.degrees(self.yaw_rate)
        # cd_servo = self.kd_servo * (-yaw_rate)

        # servo_pd = int(-(cp_servo + cd_servo))

        servo_pd = int(-cp_servo)
        u_servo = self.servo_middle + servo_pd
        # print(servo_pd, type(servo_pd))

        if u_servo > self.servo_range[1]:
            u_servo = self.servo_range[1]
        elif u_servo < self.servo_range[0]:
            u_servo = self.servo_range[0]

        # end = time.time()
        # print(end - start)
        return int(u_servo)
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
    docking = Docking()
    rospy.init_node('Docking')
    
    if not docking.webcam.isOpened(): # 캠이 연결되지 않았을 경우 # true시 캠이 잘 연결되어있음
            print("Could not open webcam")
            exit()
            
    while docking.webcam.isOpened():
        u_servo, u_thruster, size = docking.run()
        if (cv.waitKey(1) & 0xFF == 27) or size == 100: # esc버튼 누르면 창 꺼짐
            docking.finish = True
            break
            
    rate = rospy.Rate(10) # 10Hz
    
    try:
        while not rospy.is_shutdown():
            docking.publish_value(u_servo, u_thruster)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()