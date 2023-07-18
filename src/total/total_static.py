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

class Total_Static:
    def __init__(self):
        #Goal
        self.remained_waypoint = []
        gnss_waypoint = rospy.get_param("waypoints")
        for waypoint in gnss_waypoint:
            n,e,_ = gc.enu_convert(waypoint)
            enu_waypoint = [n, e]
            self.remained_waypoint.append(enu_waypoint)

        self.goal_x = self.remained_waypoint[0][1]
        self.goal_y = self.remained_waypoint[0][0]

        self.goal_range = rospy.get_param("goal_range")
        self.distance_goal = 0
        self.psi_candidate = [] # cal_error_angle에서 사용되는 11,1 array
        self.psi_desire = 0
        
        #My Boat
        self.psi = 0 # 자북 기준 heading 각도
        self.psi_queue = []  # 헤딩을 필터링할 이동평균필터 큐
        self.filter_queue_size = rospy.get_param("filter_queue_size")  # 이동평균필터 큐사이즈
        self.yaw_rate = 0 # 각가속도

        self.boat_x = 0 # x
        self.boat_x_queue = []  # boat_x을 필터링할 이동평균필터 큐
        self.boat_y = 0 # y
        self.boat_y_queue = []  # boat_y을 필터링할 이동평균필터 큐

        self.servo_range = rospy.get_param("servo_range")
        self.servo_middle = int((self.servo_range[0] + self.servo_range[1]) / 2) 
        self.u_servo = self.servo_middle
        self.u_thruster = rospy.get_param("thruster")

        #PID Control
        self.errSum = 0
        self.kp_servo = rospy.get_param("kp_servo")
        self.kd_servo = rospy.get_param("kd_servo")
        
        #Lidar
        self.obstacles = [] 
        self.angle_min = 0.0 
        self.angle_increment = 0.0 
        self.ranges = []
        self.danger_ob = {}

        #ROS
        # sub
        self.yaw_rate_sub = rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback, queue_size=1)
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_position_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)
        # self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)

        # pub
        self.end = False
        self.finish = False
        self.end_pub = rospy.Publisher("/end_check", Bool, queue_size=1)
        self.finish_pub = rospy.Publisher("/finish_check", Bool, queue_size=1)
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=1)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=1)
        self.end = False
        self.finish = False
        
        # rviz pub
        self.end = False
        self.end_pub = rospy.Publisher("/end_check", Bool, queue_size=1)
        self.psi_pub  = rospy.Publisher("/pis",Float64, queue_size=1)
        self.desire_pub = rospy.Publisher("/pis_desire", Float64, queue_size=1)
        
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

    # callback function
    def obstacle_callback(self, msg):
        self.obstacles = msg.obstacle

    # publish function
    def boat_position_pub(self):
        boat_position = Point()
        boat_position.x = self.boat_x
        boat_position.y = self.boat_y
        self.boat_pos_pub.publish(boat_position)

    def rviz_publish(self):
        self.psi_pub.publish(self.psi)
        self.psi_desire_pub.publish(self.psi_desire)
        self.end_pub.publish(self.end)
        

    # 이동 평균 필터
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

    def next(self):
        del self.remained_waypoint[0]
        self.goal_x=self.remained_waypoint[0][1]
        self.goal_y=self.remained_waypoint[0][0]


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
        # print(f"vertor_desired: {self.vector_desired}\n")
        return self.vector_desired
    
    def rerange_angle(self):
        output_angle = []
        
        self.psi_candidate = self.delete_vector_inside_obstacle(self.make_detecting_vector())
        for i in range(len(self.psi_candidate)):
            if self.psi_candidate[i][2] >= 180:
                output_angle.append(-180 + abs(self.psi_candidate[i][2]) % 180)
            elif self.psi_candidate[i][2] <= -180:
                output_angle.append(180 - abs(self.psi_candidate[i][2]) % 180)
            else:
                output_angle.append(self.psi_candidate[i][2])
        # print(f"rerange angle: {output_angle}\n")
        
        return output_angle
    
    def servo_pid_controller(self):
        self.error_angle = self.choose_velocity_vector(self.rerange_angle())
        cp_servo = self.kp_servo * self.error_angle
        # print(f"error_angle: {self.error_angle}\n")
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
              #candidate: {self.psi_candidate}\n \

def main():
    rospy.init_node("Total_Static", anonymous=False)
    rate = rospy.Rate(10) # 10 Hz
    total_static = Total_Static()
    count = 0
    
    while not total_static.is_all_connected():
        rospy.sleep(0.2)
        print("\n{:<>70}".format(" All Connected !"))

    while not rospy.is_shutdown():
        total_static.cal_distance_goal()
        total_static.print_state()

        total_static.make_detecting_vector()

        total_static.end = total_static.end_check()
        if total_static.end:
            total_static.next()
            count+=1
            print("arrive")
            rospy.sleep(3)
        else:
            pass

        if count == 0:
            total_static.control_publish()
        
        # if count == 1:
        #     print("11111111111")
        #     if total_static.finish == True:
        #         total_static.control_publish()
            
        # if count == 2:
        #     print("2222222222222")
        #     total_static.control_publish()

        if count == 3:
            print("33333333333333333")
            total_static.servo_pub.publish(total_static.servo_middle)
            total_static.thruster_pub.publish(1500)
            print("-------------Finished---------------")

        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()