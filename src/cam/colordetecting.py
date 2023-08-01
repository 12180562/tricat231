#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import cv2 as cv
import numpy as np
import cam.markDetection as markDetection

class ColorDetect :
    def __init__(self):
        
        # detecting_color : Blue = 1, Green = 2, Red = 3, Orange = 4, Black = 5
        self.detecting_color = 1
        # detecting_shape : Circle = 0, Triangle = 3, Rectangle = 4, cross = 12
        self.detecting_shape = 0

        self.webcam = cv.VideoCapture(0)

        # create window for trackbars
        cv.namedWindow('controller')

        # create trackbars for color change

        cv.createTrackbar('lowH', 'controller', 0, 179, self.nothing)
        cv.createTrackbar('highH', 'controller', 255, 179, self.nothing)

        cv.createTrackbar('lowS', 'controller', 0, 255, self.nothing)
        cv.createTrackbar('highS', 'controller', 255, 255, self.nothing)

        cv.createTrackbar('lowV', 'controller', 0, 255, self.nothing)
        cv.createTrackbar('highV', 'controller', 255, 255, self.nothing)

    # callback function for trackbar, does nothing
    def nothing(self, x):
        x
    
    def run(self):
        _, cam = self.webcam.read() 

        # 1. 영상 이미지 전처리
        hsv_image = markDetection.image_preprocessing(cam)

        # 2. get current positions of the trackbars
        lower_color = np.array([cv.getTrackbarPos('lowH', 'controller'), cv.getTrackbarPos('lowS', 'controller'), cv.getTrackbarPos('lowV', 'controller')])
        upper_color = np.array([cv.getTrackbarPos('highH', 'controller'), cv.getTrackbarPos('highS', 'controller'), cv.getTrackbarPos('highV', 'controller')])
        mask = cv.inRange(hsv_image, lower_color, upper_color)

        # 3. 형성된 마스크에서 외곽선 검출
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = np.array(contours)
        min_area = 5000
        _, cam = markDetection.shape_and_label(self.detecting_shape, cam, contours, min_area)

        images = mask

        return images, cam

def main():
    detect = ColorDetect()
    
    if not detect.webcam.isOpened():
        print("Could not open webcam")
        exit()
        
    while detect.webcam.isOpened():
        images, cam = detect.run()
        cv.imshow("controller", images)
        # cv.imshow("cam", cam)
        if (cv.waitKey(1) & 0xFF == 27):
            break       


if __name__=="__main__":
    main()
