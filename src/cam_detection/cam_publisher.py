#!/usr/bin/env python
#-*- coding:utf-8 -*-

# publish node

import cv2 as cv
import numpy as np

import sys, os
import rospy
from std_msgs.msg import Float64, UInt16

#sys.path.append('home/joohyun/catkin_ws/src/cam_detection/src/markDetection.py/')
import markDetection

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
#from cam_detection.msg import cam_msg   # publish 할 메세지 import

class Docking :
    def __init__(self):
        self.u_servo = 93
        self.u_thruster = 1500

        self.servo_pub = rospy.Publisher("/u_servo_cam" ,UInt16, queue_size= 1)
        self.thruster_pub = rospy.Publisher("/u_thruster_cam",UInt16, queue_size= 1)
    
    def publish_value(self,servo_angle,thruster_value) :
        #rate = rospy.Rate(10)
        print("함수들어옴")
        u_servo = servo_angle
        u_thruster = thruster_value
        # 터미널에 출력
        rospy.loginfo("servo_angle = %d", u_servo)
        rospy.loginfo("thruster = %d", u_thruster)
        # 정해둔 주기(hz)만큼 일시중단
        #rate.sleep()
        self.servo_pub.publish(int(u_servo))
        self.thruster_pub.publish(int(u_thruster))


def main() :
    docking = Docking()
    # 퍼블리시 노드 camera 초기화
    rospy.init_node('camera', anonymous=True)
    rate = rospy.Rate(10)

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

    while webcam.isOpened():
        _, cam = webcam.read() # webcam으로 연결된 정보 읽어오기
        raw_image = cam
        img0 = markDetection.mean_brightness(raw_image) # 평균 밝기로 보정하는 함수
        img = cv.GaussianBlur(img0, (5, 5), 0) # 가우시안 필터 적용 # (n,n) : 가우시안 필터의 표준편차. 조정하면서 해야 함
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV) # BGR 형식의 이미지를 HSV 형식으로 전환
        mask = markDetection.color_filtering(detecting_color, hsv_image)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) # 컨투어 검출
        contours = np.array(contours)
    #    contours = contours.astype(np.float)
        contour_info, raw_image = markDetection.shape_and_label(detecting_shape, raw_image, contours)
        cv.imshow("CONTROLLER", raw_image)
        servo_angle, thruster_value = markDetection.move_with_largest(contour_info, raw_image.shape[1]) # return : servo, thruster

        docking.publish_value(servo_angle,thruster_value)
        #rospy.spin()
        if cv.waitKey(0) & 0xFF == 27: # esc버튼 누르면 창 꺼짐
            exit()
        

if __name__=="__main__":
    main()

