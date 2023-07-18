#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import math
import rospy
import numpy as np

from geometry_msgs.msg import Point
from std_msgs.msg import Float64, UInt16, Bool
from sensor_msgs.msg import Imu, LaserScan
from tricat231_pkg.msg import ObstacleList

from utils import gnss_converter as gc

class Static:
    def __init__(self):
        #Goal
        self.goal_y, self.goal_x, _ = gc.enu_convert(rospy.get_param("autonomous_goal")) # Goal coordinate (x,y)

        self.goal_range = rospy.get_param("goal_range")
        self.distance_goal = 0

        self.psi_goal = 0

        #My Boat
        self.psi = 0
        self.psi_queue = []  # 헤딩을 필터링할 이동평균필터 큐
        self.filter_queue_size = rospy.get_param("filter_queue_size")  # 이동평균필터 큐사이즈
        self.yaw_rate = 0

        self.boat_x = 0
        self.boat_x_queue = []  # boat_x을 필터링할 이동평균필터 큐
        self.boat_y = 0
        self.boat_y_queue = []  # boat_y을 필터링할 이동평균필터 큐
        
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
        self.yaw_rate_sub = rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback, queue_size=1)
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_position_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        # self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)
        
        self.psi_pub  = rospy.Publisher("/pis", Float64, queue_size=1)
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=1)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=1)

        #Static Obstacle
        self.angle_number = rospy.get_param("angle_number")
        self.detecting_angle = rospy.get_param("detecting_angle")
        self.margin = rospy.get_param("margin")
        self.detecting_points = np.zeros([self.angle_number+1,3])
        self.reachableVel_global_all = []
        self.vector_desired = 0
        self.error_angle = 0

    def yaw_rate_callback(self, msg):
        self.yaw_rate = msg.angular_velocity.z

    def heading_callback(self, msg):
        self.psi = self.moving_avg_filter(self.psi_queue, self.filter_queue_size, msg.data)
        # self.psi = msg.data
        
    def boat_position_callback(self, msg):
        self.boat_y = self.moving_avg_filter(self.boat_y_queue, self.filter_queue_size, msg.x)
        self.boat_x = self.moving_avg_filter(self.boat_x_queue, self.filter_queue_size, msg.y)
        # self.boat_y = msg.x
        # self.boat_x = msg.y

    def obstacle_callback(self, msg):
        self.obstacles = msg.obstacle
        
    def moving_avg_filter(self, queue, queue_size, input, use_prev=False):         
        if not use_prev:
            if len(queue) >= queue_size:
                queue.pop(0)
            queue.append(input)
        return sum(queue) / float(len(queue))
    
    # def lidar_callback(self, data):
    #     self.angle_min = data.angle_min
    #     self.angle_increment = data.angle_increment
    #     self.ranges = data.ranges

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
        self.distance_goal = math.hypot(self.boat_x-self.goal_x, self.boat_y-self.goal_y)

    def end_check(self):
        self.cal_distance_goal()
        return self.distance_goal <= self.goal_range


    def make_detecting_vector(self):
        detecting_points = np.zeros([self.angle_number+1,3])
        angle_list = [self.psi]

        for i in range(int(self.angle_number/2)):
            angle_list.append(self.psi + (i+1)*self.detecting_angle/(self.angle_number/2))
            angle_list.append(self.psi - (i+1)*self.detecting_angle/(self.angle_number/2))

        for j in range(len(angle_list)):
            detecting_points[j][0] = math.cos(angle_list[j])
            detecting_points[j][1] = math.sin(angle_list[j])
            detecting_points[j][2] = angle_list[j]

        return detecting_points
    
    def delete_vector_inside_obstacle(self, reachableVel_global_all):
        static_OB_data=[]
        
        for i in self.obstacles:            
            static_OB_data.append(i.begin.x)
            static_OB_data.append(i.begin.y)
            static_OB_data.append(i.end.x)
            static_OB_data.append(i.end.x)

        pA = np.array([self.boat_x,self.boat_y])
        delta_t = 1
        obstacle_number = 0

        while (obstacle_number) != len(static_OB_data):
            
            # obstacle_point_x = [static_OB_data[obstacle_number][0],static_OB_data[obstacle_number+1][0]] 
            # obstacle_point_y = [static_OB_data[obstacle_number][1],static_OB_data[obstacle_number+1][1]]
            
            obstacle_point_x = [static_OB_data[obstacle_number],static_OB_data[obstacle_number+2]]
            obstacle_point_y = [static_OB_data[obstacle_number+1],static_OB_data[obstacle_number+3]]
            obstacle_number+=4

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
                    reachableVel_global_all[i][2] = 180
                else:
                    pass
                # after_delta_t_x = reachableVel_global_all_after_delta_t[i][0]+self.boat_x
                # after_delta_t_y = reachableVel_global_all_after_delta_t[i][1]+self.boat_y
                
                # if (self.boat_x-after_delta_t_x) == 0 and (self.boat_y-after_delta_t_y) < 0:
                #     vector_slope = 9999

                # elif (self.boat_x-after_delta_t_x) == 0 and (self.boat_y-after_delta_t_y) > 0:
                #     vector_slope = -9999
                    
                # else:
                #     vector_slope = (self.boat_y-after_delta_t_y)/(self.boat_x-after_delta_t_x)

                # if self.get_crosspt(slope, vector_slope, obstacle_point_x[0], obstacle_point_y[0],obstacle_point_x[1], obstacle_point_y[1], self.boat_x, self.boat_y, after_delta_t_x, after_delta_t_y):
                #     reachableVel_global_all[i][2] = 180
                # else:
                #     pass

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
                        # print(True)
                        return True # True가 맞음
                    else:
                        return False # False가 맞음
                else:
                    return False # False가 맞음

            elif OS_pos_x >= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (min(x_point)-self.margin) <= cross_x <= (max(x_point)+self.margin) and (min(y_point)-self.margin) <= cross_y <= (max(y_point)+self.margin):
                    if after_delta_t_x <= cross_x <= OS_pos_x and OS_pos_y <= cross_y <= after_delta_t_y:
                        # print(True)
                        return True
                    else:
                        return False
                else:
                    return False

    def choose_velocity_vector(self,reachableVel_global_all):
        minNum = 0
        for n in range(len(reachableVel_global_all)):
            absNum = abs(reachableVel_global_all[n] - math.degrees(math.atan2(self.goal_y - self.boat_y, self.goal_x - self.boat_x)))

            if absNum > minNum:
                minNum = absNum
                self.vector_desired = reachableVel_global_all[n]

        return self.vector_desired

    def cal_err_angle(self):
        output_angle = []
        
        self.psi_candidate = self.delete_vector_inside_obstacle(self.make_detecting_vector())
        for i in range(len(self.psi_candidate)):
            if self.psi_candidate[i][2] >= 180:
                output_angle.append(-180 + abs(self.psi_candidate[i][2]) % 180)
            elif self.psi_candidate[i][2] <= -180:
                output_angle.append(180 - abs(self.psi_candidate[i][2]) % 180)
            else:
                output_angle.append(self.psi_candidate[i][2])

        return output_angle
    
    def servo_pid_controller(self):
        self.error_angle = self.choose_velocity_vector(self.cal_err_angle())
        cp_servo = self.kp_servo * self.error_angle

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
    
    def print_state(self):
        print(f"------------------------------------\n \
              distance, thruster : {self.distance_goal}, {self.u_thruster}\n \
              my xy : {self.boat_x}, {self.boat_y}\n \
              goal xy : {self.goal_x}, {self.goal_y}\n \
              psi, desire : {round(self.psi,2)}, {round(self.vector_desired,2)}\n \
              servo : {self.servo_pid_controller()-93}\n")

def main():
    rospy.init_node("Static", anonymous=False)
    rate = rospy.Rate(10) # 10 Hz
    static = Static()

    while not static.is_all_connected():
        rospy.sleep(0.2)
        print("\n{:<>70}".format(" All Connected !"))

    while not rospy.is_shutdown():
        static.cal_distance_goal()
        static.print_state()

        if static.end_check():
            static.servo_pub.publish(static.servo_middle)
            static.thruster_pub.publish(1500)
            print("FINISHED")
        else:

            static.make_detecting_vector()
            static.control_publish()
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()