#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
from math import pow, sqrt, cos, sin

"""lidar_converter 등에 사용할 '한 점' 혹은 '한 벡터' 정의 클래스"""
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @classmethod
    def polar_to_cartesian(cls, r, phi):
        """극좌표계 표현(각도, 거리) -> 데카르트 좌표계 표현(x, y)해 클래스 인스턴스 생성"""
        return cls(r * cos(phi), r * sin(phi))

    def __add__(self, p2):
        """연산자 '+' 재정의: 두 벡터의 덧셈"""
        return Point(self.x + p2.x, self.y + p2.y)

    def __sub__(self, p2):
        """연산자 '-' 재정의: 두 벡터의 뺄셈"""
        return Point(self.x - p2.x, self.y - p2.y)

    def __mul__(self, c):
        """연산자 '*' 재정의: 벡터의 크기 상수배"""
        return Point(self.x * c, self.y * c)

    def __div__(self, d):
        """연산자 '/' 재정의: 벡터의 크기 상수배"""
        return Point(self.x / d, self.y / d) if d != 0 else Point(0, 0)

    def __eq__(self, p2):
        """연산자 '==' 재정의: 두 벡터(점)가 같은 값인가"""
        return self.x == p2.x and self.y == p2.y

    def dist_from_origin(self):
        """
        (1) 원점으로부터 해당 점까지의 거리
        (2) 벡터의 크기
        """
        return sqrt(pow(self.x, 2.0) + pow(self.y, 2.0))

    def dist_btw_points(self, p2):
        """두 점 간의 거리"""
        return sqrt(pow(self.x - p2.x, 2.0) + pow(self.y - p2.y, 2.0))

    def dot(self, p):
        """두 벡터의 내적"""
        return self.x * p.x + self.y * p.y

    def perpendicular(self):
        """원점 대칭"""
        return Point(-self.y, self.x)



"""lidar_converter에 사용할 '점의 집합'(그룹) 정의 클래스"""
class PointSet:
    def __init__(self):
        self.point_set = []  # 그룹 내 점의 목록(point_class.py의 Point 클래스)
        self.set_size = 0  # 그룹 내 점의 개수
        self.begin = None  # 그룹의 시작점
        self.end = None  # 그룹의 끝
        # self.boat_x, self.boat_y = 0, 0  # 현재 보트 위치
        # self.boat_set = [self.boat_x,self.boat_y]

    def input_point_set(self, ps):
        """특정 PointSet의 각 인스턴스를 설정함

        Args:
            ps (PointSet): 등록 대상 PointSet
        """
        self.point_set = ps
        self.set_size = len(ps)
        self.begin = ps[0]
        self.end = ps[-1]

    def append_point(self, p):
        """해당 PointSet(그룹)에 한 점을 추가

        Args:
            p (Point): 추가할 포인트 개체
        """
        if len(self.point_set) == 0:
            self.begin = p  # 그룹의 첫 점이면 begin으로 설정

        self.point_set.append(p)  # 점 1개 더 등록
        self.set_size += 1  # 점의 개수 1개 증가
        self.end = p  # 추가된 점은 끝점으로 설정.
        # 위 코드는 self.point_set[-1]로 해도 동일. (self.end)와 p의 아이디(id())가 같음. 같은 포인터

    def projection(self, p):
        """특정 점을 이 PointSet로의 투영점 위치 계산a.dist_squared_from_origin() != 0:
        Args:
            p (Point): 투영할 포인트 개체
        Returns:
            projection (list): 투영점 좌표
        """
        a = self.end - self.begin  # PointSet의 첫 점과 끝 점 이은 벡터
        b = p - self.begin  # 첫 점과 특정 점을 이은 벡터

        if sqrt(a.dist_from_origin()) != 0:
            projection = self.begin + a * (a.dot(b) / sqrt(a.dist_from_origin()))  # 내적 이용
        else:  # PointSet의 크기가 0 혹은 1인 경우
            projection = self.begin

        return projection

    def dist_to_point(self, p):
        """특정 점과 이 PointSet까지의 거리

        특정 점의 이 PointSet의 '첫점 ~ 끝점 이은 선분' 위로 내린 수선의 발까지 거리로 계산함

        Args:
            p (Point): 거리를 계산할 포인트
        Returns:
            distance_to_point (float): 점과 그룹 간 거리
        """
        # print("p", p.x)
        # print("원점까지 거리", p.dist_from_origin())
        # print("수선의 발 (", self.projection(p).x, ", ", self.projection(p).y, ")")
        # print("벡터 (", ( p - self.projection(p)).x, ", ", ( p - self.projection(p)).y)
        distance_to_point = (p - self.projection(p)).dist_from_origin()

        return distance_to_point

    def dist_begin_to_end(self):
        """PointSet의 길이: 첫 점과 끝 점까지 거리"""
        return sqrt(pow(self.begin.x - self.end.x, 2) + pow(self.begin.y - self.end.y, 2))
    

    # def inclination(self):

    #     return (self.end.y - self.begin.y) / (self.end.x - self.begin.x)
    
    def point_angle_calc(self):

        return np.arctan(self.y/self.x)
    
    # def end_point_angle_calc(self):

    #     return np.arctan(self.end.y/self.end.x)
