#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import rospy
from math import hypot, sin, cos, radians

from utils import gnss_converter as gc

class Total_Static:
    def __init__(self):
        #Goal
        self.remained_waypoint = []
        gnss_waypoint = rospy.get_param("waypoints")
        for waypoint in gnss_waypoint:
            n,e,_ = gc.enu_convert(waypoint)
            enu_waypoint = [n, e]
            self.remained_waypoint.append(enu_waypoint)

        self.goal_x = self.remained_waypoint[0][0] # x=0 y=1이 맞음
        self.goal_y = self.remained_waypoint[0][1] 

        self.goal_range = rospy.get_param("goal_range")
        self.distance_goal = 0
        self.goal_count = 0
        #My Boat
        self.boat_x = 0 
        self.boat_y = 0
        self.angle_list = []

    def cal_distance_goal(self):
        self.distance_goal = hypot(self.boat_x-self.goal_x, self.boat_y-self.goal_y)

    def end_check(self):
        self.cal_distance_goal()
        return self.distance_goal <= self.goal_range

    def next(self):
        if self.goal_count == len(self.remained_waypoint):
            return False
        else:
            self.goal_count += 1
            self.goal_x=self.remained_waypoint[self.goal_count][0] # x = 0
            self.goal_y=self.remained_waypoint[self.goal_count][1] # y = 1
            return True

    def generate_candidate_angle(self):
        self.angle_list = [self.psi]

        for i in range(self.angle_number):
            self.angle_list.append(self.psi + (i+1) * (self.detecting_angle/self.angle_number))
            self.angle_list.append(self.psi - (i+1) * (self.detecting_angle/self.angle_number))

        return self.angle_list

    def append_non_cross_angle(self, candidate):
        static_OB_data = []
        ob_slope = []
        boat_slope = []

        for i in self.obstacles:
            begin_x = self.boat_x + (-i.begin.x) * cos(radians(self.psi)) - i.begin.y * sin(radians(self.psi))
            begin_y = self.boat_y + (-i.begin.x) * sin(radians(self.psi)) + i.begin.y * cos(radians(self.psi))
            end_x = self.boat_x + (-i.end.x) * cos(radians(self.psi)) - i.end.y * sin(radians(self.psi))
            end_y = self.boat_y + (-i.end.x) * sin(radians(self.psi)) + i.end.y * cos(radians(self.psi))
            static_OB_data.append(begin_x)
            static_OB_data.append(begin_y)
            static_OB_data.append(end_x)
            static_OB_data.append(end_y)

        for obstacle_number in range(len(static_OB_data)):
            obstacle_point_x = [static_OB_data[obstacle_number],static_OB_data[obstacle_number+2]]
            obstacle_point_y = [static_OB_data[obstacle_number+1],static_OB_data[obstacle_number+3]]
            obstacle_number += 4

            if obstacle_point_x[0] > obstacle_point_x[1]:
                obstacle_point_x.reverse()
                obstacle_point_y.reverse()
            #기울기를 구할때 큰값빼기 작은값을 함
            #갈때는 오케인데 돌아올때는 부호가 바뀌지 않나?
            ob_slope.append(obstacle_point_y[1]-obstacle_point_y[0])/((obstacle_point_x[1]-obstacle_point_x[0]) + 0.00000001)

        for i in range(self.angle_number):
            after_delta_t_x = (cos(radians(candidate[i])) + self.boat_x) * self.delta_t
            after_delta_t_y = (sin(radians(candidate[i])) + self.boat_y) * self.delta_t

            boat_slope.append(after_delta_t_y-self.boat_y)/((after_delta_t_x-self.boat_x)+0.00000001)

        if self.cross_check():
            non_cross_vector.append(candidate)
        else:
            pass

        if len(non_cross_vector) == 0:
            non_cross_vector.append(non_cross_vector[self.angle_number])
            non_cross_vector.append(non_cross_vector[self.angle_number+1])