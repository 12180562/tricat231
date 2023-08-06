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

        cv.createTrackbar('lowH', 'controller', 0, 255, self.nothing)
        cv.createTrackbar('highH', 'controller', 255, 255, self.nothing)

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

        return mask

def main():
    detect = ColorDetect()
    
    if not detect.webcam.isOpened():
        print("Could not open webcam")
        exit()
        
    while detect.webcam.isOpened():
        mask = detect.run()
        cv.imshow("controller", mask)
        # cv.imshow("cam", cam)
        if (cv.waitKey(1) & 0xFF == 27):
            break       


if __name__=="__main__":
    main()
