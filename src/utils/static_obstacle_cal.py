#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""hyo.py에 사용되는 모듈"""

import numpy as np
from math import hypot, sqrt
import queue


class test:
    def __init__(self, boat_x, boat_y, vector_x, vector_y, start_x, start_y, end_x, end_y, range):
        self.boat_x = boat_x
        self.boat_y = boat_y
        self.vector_x = vector_x
        self.vector_y = vector_y

        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y

        self.sub_x = self.end_x - self.start_x
        self.sub_y = self.end_y - self.start_y

        self.boat_con = self.vector_y*self.boat_x - self.vector_x*self.boat_y
        self.ob_con = self.start_y*self.end_x - self.end_y*self.start_x

        self.range = range
        self.point = None

    def cal_cross(self):
        A = np.array([[self.vector_y, -self.vector_x], [self.sub_y, self.sub_x]])

        B = np.array([self.boat_con, self.ob_con])

        if np.linalg.det(A) != 0:
            X = np.linalg.solve(A, B)
            self.point = [X[0], X[1]]
        else:
            raise ValueError("Cannot calculate the cross point.")

        return self.point

    def cal_dist(self):
        if self.point is None:
            raise ValueError("No cross point. Call cal_cross() first.")
        
        return sqrt((self.point[0] - self.boat_x)**2 + (self.point[1] - self.boat_y)**2)

    def cross_check(self):
        try:
            self.cal_cross()
        except ValueError:
            return False

        if self.start_x <= self.point[0] <= self.end_x and \
           self.start_y <= self.point[1] <= self.end_y and \
           self.cal_dist() <= self.range:
            return True
        else:
            return False



# class staticOB_cal:
#     def __init__(self, boat_x, boat_y, vector_x, vector_y, start_x, start_y, end_x, end_y, range):
#         # 보트의 위치 및 벡터
#         self.boat_x = boat_x 
#         self.boat_y = boat_y
#         self.vector_x = vector_x
#         self.vector_y = vector_y

#         # 장애물의 시작점과 끝점
#         self.start_x = start_x
#         self.start_y = start_y
#         self.end_x = end_x
#         self.end_y = end_y

#         # 장애물의 x 및 y 범위
#         self.sub_x = self.end_x-self.start_x
#         self.sub_y = self.end_y-self.start_y

#         # 보트와 장애물의 위치에 대한 행렬식 계산
#         self.boat_con = self.vector_x*self.boat_y - self.vector_y*self.boat_x
#         self.ob_con = self.end_y*self.start_x - self.start_y*self.end_x

#         # 교차점 검사에 사용되는 거리 범위
#         self.range = range

#         # 교차점 초기화
#         self.point = [0, 0]

#     def cal_cross(self):
#         # 보트 벡터와 장애물 위치를 사용해 행렬 A 생성
#         A = np.array([[self.vector_x, -self.vector_y], [self.sub_x, self.sub_y]])

#         # 보트와 장애물에 대한 행렬식을 사용해 행렬 B 생성
#         B = np.array([self.boat_con, self.ob_con])

#         # 행렬 A의 행렬식이 0이 아니라면 (즉, 행렬 A가 역행렬을 가질 수 있다면)
#         if np.linalg.det(A) != 0: 
#             # 행렬 A의 역행렬과 행렬 B를 곱하여 교차점 계산
#             X = np.dot(np.linalg.inv(A), B)
#             self.point = [X[0], X[1]]

#     def cal_dist(self):
#         # 보트와 교차점 사이의 유클리디안 거리 계산
#         # return sqrt((self.point[0] - self.boat_x)**2 + (self.point[1] - self.boat_y)**2)
#         return hypot(self.point[0] - self.boat_x, self.point[1] - self.boat_y)

#     def cross_check(self):
#         # 교차점 계산
#         self.cal_cross()

#         # 교차점이 장애물 내부에 있고, 보트와의 거리가 주어진 범위 내에 있는지 확인
#         if self.start_x < self.point[0] < self.end_x and self.start_y < self.point[1] < self.end_y:
#             if self.cal_dist() < self.range:
#                 return True
#             else:
#                 return False
#         else:
#             return False


# class staticOB_cal:
#     def __init__(self, boat_x, boat_y, vector_x, vector_y, start_x, start_y, end_x, end_y, range):
#         # boat position
#         self.boat_x = boat_x 
#         self.boat_y = boat_y
#         # boat vector
#         self.vector_x = vector_x
#         self.vector_y = vector_y

#         # obstacle
#         self.start_x = start_x
#         self.start_y = start_y
#         self.end_x = end_x
#         self.end_y = end_y

#         # matrix cal
#         self.sub_x = self.end_x-self.start_x
#         self.sub_y = self.end_y-self.start_y
#         self.boat_con = self.vector_x*self.boat_y + self.vector_y*self.boat_x
#         self.ob_con = self.end_x*self.start_y - self.start_x*self.end_y
#         self.point = [0, 0]

#         # cross check
#         self.range = range
#         self.dist = 0

#     def cal_cross(self):
#         A = Matrix(self.vector_x, -self.vector_y, self.sub_x, self.sub_y)
#         if(A.det()==0): # 부정이나 불능인 경우 예외 처리
#             return False
#         else:
#             B = Matrix().twobyone(self.boat_con,self.ob_con,0,0)
#             X = np.dot(A.twobytwo_reverse(),B)
#             self.point = [X[0],X[1]]
#             return self.point
    
#     def cal_dist(self):
#         return sqrt((self.point[0] - self.boat_x)**2+(self.point[1] - self.boat_y)**2)
        
#     def cross_check(self):
#         self.cal_cross()
#         if(self.start_x < self.point[0] < self.end_x and self.start_y < self.point[1] < self.end_y):
#             if(self.cal_dist() < self.range):
#                 return True
#             else:
#                 return False
#         else:
#             return False
            
# class Matrix:
#     def __init__(self, a, b, c, d):
#         self.a = a
#         self.b = b
#         self.c = c
#         self.d = d

#     def twobytwo(self):
#         return np.array([self.a, self.b],[self.c, self.d])
    
#     def twobyone(self):
#         return np.array([self.a],[self.b])
    
#     def det(self):
#         return self.a*self.d-self.b*self.c
    
#     def twobytwo_reverse(self):
#         return (1/self.det())*np.array([self.d,-self.b],[self.c,self.a])


'''
코드 설명[작성:J]

#-- 생성한 벡터를 통해 만들어지는 직선의 방정식 --#

한 점(x_s,y_s)과 하나의 벡터(a,b)를 알고 있는 경우 직선의 방정식: (x-x_s)/a = (y-y_s)/b 
이를 정리하면 a*y - b*x = a*y_s - b*x_s
여기서 한 점은 선박의 위치이므로 boat_x,boat_y와 동일

#-- obstacle의 정보를 통해 만들어지는 직선의 방정식 --#
    
두 점([start_x, start_y], [end_x, end_y])를 알고 있는 경우 직선의 방정식: y = ((end_y - end_x)/(end_x - start_x))*(x - start_x) + start_y
정리하면 (start_y - end_y)*x + (end_x - start_x)*y = end_x*start_y - start_x*end_y
여기서 두 점은 clustering된 lidar정보임

 #-- 연립 일차 방정식 행렬 표현 --#

여기서부터는 numpy를 이용하여 행렬을 표현, numpy에 관한 설명은 인터넷 참조 바람

np.array([a, -b],[start_y - end_y, end_x - start_x]).dot(np.array([x],[y])) = np.array([a*y_s + b*x_s],[end_x*start_y - start_x*end_y])

연립일차방정식 AX=B에 대해서 A의 역행렬이 존재하는 경우 양변에 A의 역행렬을 곱해서 미지수 X의 해를 구할 수 있음
만약 A의 역행렬이 없으면 연립일차방정식은 불능(해가 없음) 또는 부정(해가 무수히 많음)이다.
즉, np.array([a, -b],[end_x - start_x, end_x - start_x])의 역행렬을 구해야 하며 이는 2x2 행렬이므로 다음과 같다.
        
#-- 2x2 역행렬을 구하는 방법 --#

A = np.array([a,b],[c,d])일 때 A의 역행렬은 (1/det(A))*np.array([d,-b],[c,a])
즉 (1/(a*d-b*c))*np.array([d,-b],[c,a])이고 이때 a*d-b*c가 0이 아니여야 한다.
        
이를 위의 식을 이용하여 나타내면 다음과 같음
det(A) = a*(end_x - start_x)+b*(start_y - end_y)
np.array([x],[y]) = (1/(a*(end_x - start_x)+b*(end_x - start_x)))*np.array([end_x - start_x, b],[start_y - end_y, a]).dot(np.array([a*y_s + b*x_s],[end_x*start_y - start_x*end_y])
'''

# 이동평균필터(Moving average filter)
class MovAvgFilter:
    
    # 이전 스텝의 평균
    prevAvg = 0
    # 가장 최근 n개의 값을 저장하는 큐
    xBuf = queue.Queue()
    # 참조할 데이터의 갯수
    n = 0
    
    def __init__(self, _n):
        # 초기화로 n개의 값을 0으로 둡니다.
        for _ in range(_n):
            self.xBuf.put(0)
        # 참조할 데이터의 갯수를 저장합니다.
        self.n = _n
    
    def movAvgFilter(self, x):
        # 큐의 front 값은 x_(k-n) 에 해당합니다.
        front = self.xBuf.get()
        # 이번 스텝에 입력 받은 값을 큐에 넣습니다.
        self.xBuf.put(x)
        
        avg = self.prevAvg + (x - front) / self.n     
        self.prevAvg = avg
        
        return avg   