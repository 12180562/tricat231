#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import math
import rospy
import numpy as np

from geometry_msgs.msg import Point
from std_msgs.msg import Float64, UInt16
from sensor_msgs.msg import Imu, LaserScan
from tricat231_pkg.msg import ObstacleList

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

        self.goal_x = self.remained_waypoint[0][0]
        self.goal_y = self.remained_waypoint[0][1]

        self.goal_range=rospy.get_param("goal_range")
        self.distamce_goal = 0
        
        self.psi_candidate = []

        #My Boat
        self.psi = 0
        self.yaw_rate = 0

        self.boat_x = 0
        self.boat_y = 0

        self.servo_range = rospy.get_param("servo_range")
        self.servo_middle = int((self.servo_range[0] + self.servo_range[1]) / 2) 
        self.u_servo = self.servo_middle
        
        self.u_thruster = rospy.get_param("thruster")

        #PID Control
        self.errSum=0
        self.kp_servo = rospy.get_param("kp_servo")
        self.kd_servo = rospy.get_param("kd_servo")
        
        #Lidar
        self.obstacles = []

        self.angle_min = 0.0 
        self.angle_increment = 0.0
        self.ranges = []
        self.danger_ob = {}

        #ROS
        self.yaw_rate_sub = rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback)
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_position_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        # self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)

        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)
        
        #Static Obstacle
        self.angle_number = rospy.get_param("angle_number")
        self.detecting_angle = rospy.get_param("detecting_angle")
        self.margin = rospy.get_param("margin")
        self.detecting_points = np.zeros([self.angle_number+1,3])
        self.reachableVel_global_all = []

    def yaw_rate_callback(self, msg):
        self.yaw_rate = msg.angular_velocity.z 

    def heading_callback(self, msg):
            self.psi = msg.data

    def boat_position_callback(self, msg):
            self.boat_y = msg.x
            self.boat_x = msg.y

    def obstacle_callback(self, msg):
        self.obstacles = msg.obstacle

    def lidar_callback(self, data):
        self.angle_min = data.angle_min
        self.angle_increment = data.angle_increment
        self.ranges = data.ranges

    def is_all_connected(self):
        rospy.wait_for_message("/heading", Float64)
        print("\n{:><70}".format("heading_calculator Connected "))
        rospy.wait_for_message("/enu_position", Point)
        print("\n{:><70}".format("gnss_converter Connected "))
        rospy.wait_for_message("/obstacles", ObstacleList)
        print("\n{:><70}".format("lidar_converter Connected "))
        # rospy.wait_for_message("/scan", LaserScan)
        # print("\n{:><70}".format("LiDAR Connected "))

        return True

    def cal_distance_goal(self):
            self.distance_goal=math.hypot(self.boat_x-self.goal_x,self.boat_y-self.goal_y)

    def end_check(self):
        self.cal_distance_goal()
        return self.distance_goal <= self.goal_range

    def next(self):
        del self.remained_waypoint[0]

        self.goal_x=self.remained_waypoint[0][0]
        self.goal_y=self.remained_waypoint[0][1]

    def cal_err_angle(self):
        self.psi_candidate = self.delete_vector_inside_obstacle(self.detecting_points)
        for i in range(len(self.psi_candidate)):
            if self.psi_candidate >= 180:
                output_angle = -180 + abs(self.psi_candidate) % 180
            elif self.psi_candidate <= -180:
                output_angle = 180 - abs(self.psi_candidate) % 180
            else:
                output_angle = self.psi_candidate
        return output_angle
    
    def servo_pid_controller(self):
        error_angle = self.choose_velocity_vector(self.cal_err_angle())
        cp_servo = self.kp_servo * error_angle

        yaw_rate = math.degrees(self.yaw_rate)
        cd_servo = self.kd_servo * (-yaw_rate)

        servo_pd = -(cp_servo + cd_servo)
        u_servo = self.servo_middle + servo_pd

        if u_servo > self.servo_range[1]:
            u_servo = self.servo_range[1]
        elif u_servo < self.servo_range[0]:
            u_servo = self.servo_range[0]

        return int(u_servo)
    
    def control_publish(self):
        self.servo_pid_controller()
        self.servo_pub.publish(self.u_servo)
        self.thruster_pub.publish(self.u_thruster)

    def make_detecting_vector(self):
        angle_list = [self.psi]

        for i in range(int(self.angle_number/2)):
            angle_list.append(self.psi + (i+1)*self.detecting_angle/(self.angle_number/2))
            angle_list.append(self.psi - (i+1)*self.detecting_angle/(self.angle_number/2))

        for j in range(len(angle_list)):
            self.detecting_points[j][0] = math.cos(angle_list[j])
            self.detecting_points[j][1] = math.sin(angle_list[j])
            self.detecting_points[j][2] = angle_list[j]

        return self.detecting_points

    def delete_vector_inside_obstacle(self, reachableVel_global_all):
        static_OB_data = self.obstacles
        
        pA = np.array([self.boat_x,self.boat_y])
        delta_t = 1
        obstacle_number = 0

        while (obstacle_number) != len(static_OB_data):
            obstacle_point_x = [static_OB_data[obstacle_number].x,static_OB_data[obstacle_number+1].x] 
            obstacle_point_y = [static_OB_data[obstacle_number].y,static_OB_data[obstacle_number+1].y] 

            if obstacle_point_x[0] > obstacle_point_x[1]:
                obstacle_point_x.reverse()
                obstacle_point_y.reverse()

            if (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) > 0:
                slope = 9999

            elif (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) < 0:
                slope =-9999

            else: 
                slope = (obstacle_point_y[1]-obstacle_point_y[0])/(obstacle_point_x[1]-obstacle_point_x[0])
                
            reachableVel_global_all_after_delta_t = reachableVel_global_all * delta_t 

            for i in range(len(reachableVel_global_all_after_delta_t)): 
                after_delta_t_x = reachableVel_global_all_after_delta_t[i][0]+pA[0]
                after_delta_t_y = reachableVel_global_all_after_delta_t[i][1]+pA[1]
                if (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) < 0:
                    vector_slope = 9999

                elif (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) > 0:
                    vector_slope = -9999
                else:
                    vector_slope = (pA[1]-after_delta_t_y)/(pA[0]-after_delta_t_x)

                if self.get_crosspt(slope, vector_slope, obstacle_point_x[0], obstacle_point_y[0],obstacle_point_x[1], obstacle_point_y[1], pA[0], pA[1], after_delta_t_x, after_delta_t_y):
                    del reachableVel_global_all[i]
                else:
                    pass

            obstacle_number = obstacle_number+2

        return reachableVel_global_all
        
    def get_crosspt(self, slope, vector_slope, start_x, start_y,end_x, end_y, OS_pos_x, OS_pos_y, after_delta_t_x, after_delta_t_y):

        x_point = [start_x, end_x]
        y_point = [start_y, end_y]

        if (slope) == (vector_slope): 
            return False

        else:
            cross_x = (start_x * slope - start_y - OS_pos_x * vector_slope + OS_pos_y) / (slope - vector_slope)
            cross_y = slope * (cross_x - start_x) + start_y
            # 벡터 연산 모듈로 교체 예정

            if OS_pos_x <= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (min(x_point)-self.margin) <= cross_x <= (max(x_point)+self.margin) and (min(y_point)-self.margin) <= cross_y <= (max(y_point)+self.margin):
                    if OS_pos_x <= cross_x <= after_delta_t_x and OS_pos_y <= cross_y <= after_delta_t_y:
                        print(True)
                        return True # True가 맞음
                    else:
                        return False # False가 맞음
                else:
                    return False # False가 맞음

            elif OS_pos_x >= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (min(x_point)-self.margin) <= cross_x <= (max(x_point)+self.margin) and (min(y_point)-self.margin) <= cross_y <= (max(y_point)+self.margin):
                    if after_delta_t_x <= cross_x <= OS_pos_x and OS_pos_y <= cross_y <= after_delta_t_y:
                        print(True)
                        return True
                    else:
                        return False
                else:
                    return False

    def choose_velocity_vector(self,reachableVel_global_all):
        vector_desired = 0

        for n in reachableVel_global_all:
            absNum = abs(n - math.degree(math.atan2(self.goal_y - self.boat_y, self.goal_x - self.boat_x)))

        if absNum < minNum:
            minNum = absNum
            vector_desired = n

        return vector_desired
    
    def print_state(self):
        print(f"------------------------------------\n \
              distance, thruster : {self.distance_goal}, {self.u_thruster}\n \
              my xy : {self.boat_x}, {self.boat_y}\n \
              goal xy : {self.goal_x}, {self.goal_y}\n \
              psi, desire : {self.psi}, {self.choose_velocity_vector()}\n \
              servo : {self.servo_pid_controller()-93}\n")

def main():
    rospy.init_node("Total_Static", anonymous=False)
    rate = rospy.Rate(10) # 10 Hz
    total_static = Total_Static()

    count = 0

    while not total_static.is_all_connected():
        rospy.sleep(0.2)
        print("\n{:<>70}".format(" All Connected !"))

    while not rospy.is_shutdown():
        total_static.print_state()

        total_static.make_detecting_vector()

        if total_static.end_check():
            total_static.next()
            count+=1
            print("arrive")
            rospy.sleep(3)
        else:
            pass

        if count == 0:
            total_static.control_publish()
        
        if count == 1:
            print("11111111111")
            # 카메라 켜기
            total_static.control_publish()

        if count == 2:
            print("2222222222222")
            # 카메라 끄기
            total_static.control_publish()

        if count == 3:
            print("33333333333333333")
            total_static.servo_pub.publish(total_static.servo_middle)
            total_static.thruster_pub.publish(1500)
            print("-------------Finished---------------")

        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()