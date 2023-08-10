#!/usr/bin/env python
# -*- coding:utf-8 -*-

# 전략 1
# 주어진 색상을 이용해서 detecting이 되는지를 보고, 전광판에서 도형이 detecting이 되면 해당 색상이 있는 도킹 포인트로 이동을 시도한다.
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
import cv2 as cv

from std_msgs.msg import UInt16
import cam.markDetection as markDetection

class Docking :
    def __init__(self):
        self.end = False
        self.webcam = cv.VideoCapture(0)

        # self.detecting_color = rospy.get_param("detecting_color")
        # self.detecting_shape = rospy.get_param("detecting_shape")
        # self.color_bounds = rospy.get_param("color_bounds")
        # self.min_area = rospy.get_param("min_area")
        # self.target_detect_area = rospy.get_param("target_detect_area")

        # TEST set
        # detecting_color : Blue = 1, Green = 2, Red = 3, Orange = 4, Black = 5
        self.detecting_color = 3
        self.detecting_shape = 4
        self.color_bounds = {
            "1": ([89, 50, 50], [138, 255, 255]),  # Blue
            "2": ([30, 50, 50], [80, 255, 255]),   # Green
            "3": ([0, 116, 153], [255, 255, 255]),  # Red
            "4": ([145, 197, 0], [184, 255, 255]),  # Orange
            "5": ([89, 16, 21], [151, 255, 255])   # Black
        }
        self.min_area = 1
        self.max_area = 10000
        self.count = 1
        self.docking_point  = 0

        #sub
        self.count_sub = rospy.Subscriber("/count", UInt16, self.count_callback, queue_size=10)
        #pub
        self.cam_pub = rospy.Publisher("/docking_point", UInt16, queue_size=10) # 들어갈 도킹 포인트(왼쪽 = 1, 중간 = 2, 오른쪽 = 3)

    def count_callback(self, msg):
        self.count = msg.data

    def publish_value(self) :
        self.cam_pub.publish(self.docking_point)

    def control(self):
        # webcam으로 연결된 정보 읽어오기
        _, cam = self.webcam.read()

        # 영상 이미지 전처리
        hsv_image = markDetection.image_preprocessing1(cam, brightness=False, blur=True)

        # 탐지 색상 범위에 따라 마스크 형성
        mask = markDetection.color_filtering(self.detecting_color, self.color_bounds, hsv_image)

        # 모폴로지 연산(빈 공간 완화 & 노이즈 제거) 후 형성된 마스크에서 외곽선 검출
        contours = markDetection.morph_contour(mask)

        # 외곽선에서 도형 검출
        if len(contours) > 0:
            shape, center, countour = markDetection.shape_detection(self.detecting_shape, self.max_area, self.min_area, contours)
            cam = markDetection.window1(cam, center, countour, shape)
            
            # 도형이 검출되면 해당 도킹 포인트로 이동
            docking_point = markDetection.doc_point(self.detecting_shape, shape)
        else:
            cam = cam
            docking_point = 0

        # 화면 출력
        col = markDetection.show(cam, mask)
        cv.imshow("docking", col)

        return docking_point

def main():
    docking = Docking()
    time_trigger = markDetection.TimeBasedTrigger(2)
    rospy.init_node('Docking')
    
    if not docking.webcam.isOpened(): # 캠이 연결되지 않았을 경우 # true시 캠이 잘 연결되어있음
        print("Could not open webcam")
        exit()
            
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        docking_point = docking.control()
        if time_trigger.trigger(docking.count == 1):
            docking.docking_point = docking_point
            docking.publish_value()
        else:
            docking.docking_point = 0
            docking.publish_value()
        print(docking.docking_point)

        if cv.waitKey(1) & 0xFF == 27: # 카메라 창에서 esc버튼 꺼짐
            cv.destroyAllWindows()
            break
        
        rate.sleep()        

if __name__=="__main__":
    main()