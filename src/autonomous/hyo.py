#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import rospy
import numpy as np

from geometry_msgs.msg import Point
from std_msgs.msg import Float64, UInt16
from tricat231_pkg.msg import ObstacleList

class Autonomous:
    def __init__(self):
        self.waypoint=rospy.get_param("waypoint")
        self.goal_x=self.waypoint[0][0]
        self.goal_y=self.waypoint[0][1]
        
        #obstacle
        
        self.obstacles = []
        self.boat_x,self.boat_y=0
        self.goal_range=rospy.get_param("goal_range")

        self.kp_servo = rospy.get_param("kp_servo")
        self.ki_servo = rospy.get_param("ki_servo")
        self.kd_servo = rospy.get_param("kd_servo")
        
        # detecting variables
        self.angle_list = []
        self.detecting_angle = rospy.get_param("detecting_angle")
        self.detecting_distance = rospy.get_param("detecting_distance")
        self.angle_number = rospy.get_param("angle_number") # the number of angle

        # directions
        self.psi = 0  # 자북과 heading의 각도(자북 우측 +, 좌측 -) [degree]
        self.psi_goal = 0  # 현재 선수각으로부터 goal까지 가기 위해 움직여야 할 각도. 선박 기준(-180 ~ 180)
        self.error_angle = 0  # 다음 목표까지 가기 위한 차이각
        self.psi_desire = 0  # 지구고정좌표계 기준 움직여야 할 각도

        # other fix values
        self.servo_range = rospy.get_param("servo_range")
        self.servo_middle = int((self.servo_range[0] + self.servo_range[1]) / 2)
        self.thruster_min = rospy.get_param("thruster_min")
        self.thruster_max = rospy.get_param("thruster_max")

        # other variables
        self.distance_to_goal = 0  # 다음 목표까지 남은 거리
        self.u_servo = self.servo_middle
        self.u_thruster = self.thruster_min

        #subscribers
        self.heading_sub=rospy.Subscriber("/heading",Float64,self.heading_callback)
        self.enu_position_sub =rospy.Subscriber("/enu_postion",Point,self.boat_position_callback)
        self.obstacl_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)

        self.servo_pub=rospy.Publisher("/servo",UInt16)
        self.thruster_pub=rospy.Publisher("/thruster",UInt16)

    def heading_callback(self,msg):
        self.psi=msg.data
        
    def obstacle_callback(self, msg):
        self.obstacles = msg.obstacle

    def boat_position_callback(self,msg):
        self.boat_x=msg.x
        self.boat_y=msg.y

    def make_detecting_vector(self):
        angle_list = [self.psi]
        detecting_points = np.zeros([self.angle_number+1,4])

        for i in range(int(self.angle_number/2)):
            angle_list.append(self.psi + (i+1)*self.detecting_angle/(self.angle_number/2))
            angle_list.append(self.psi - (i+1)*self.detecting_angle/(self.angle_number/2))

        for j in range(len(angle_list)):
            detecting_points[j][0] = math.cos(angle_list[j])
            detecting_points[j][1] = math.sin(angle_list[j])
            detecting_points[j][3] = angle_list[j]

        return detecting_points

    def delete_vector_inside_obstacle(self, reachableVel_global_all, position, static_obstacle_info):
    
        #static_OB_data = static_obstacle_info
        #static_OB_data = [[50,100],[100,100],[100,100],[100,50],[100,50],[50,50],[50,50],[50,100],[-50,-100],[-100,-100],[-100,-100],[-100,-50],[-100,-50],[-50,-50],[-50,-50],[-50,-100]]
        #static_OB_data = [[46.97,153.03],[82.32,117.68],[82.32,117.68],[-117.68,-82.32],[-117.68,-82.32],[-153.03,-46.97],[153.03,46.97],[117.68,82.32],[117.68,82.32],[-82.32,-117.68],[-82.32,-117.68],[-46.97,-153.03]]
        # static_OB_data = [[0, 0], [50, 0], [50, 0], [50, 50], [50, 50], [25, 50], [25, 50], [25, 150], [25, 150], [50, 150], [50, 150], [50, 175], [50, 175], [0, 175],[175, 0], [125, 0], [125, 0], [125, 75], [125, 75], [75, 75], [75, 75], [75, 125], [75, 125], [100, 125], [100, 125], [100, 175], [100, 175], [150, 175]]
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
            result = []

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
                    reachableVel_global_all[i][2] = reachableVel_global_all[i][2]-50
                else:
                    pass

            obstacle_number = obstacle_number+2

        return reachableVel_global_all
        
    def get_crosspt(self, slope, vector_slope, start_x, start_y,end_x, end_y, OS_pos_x, OS_pos_y, after_delta_t_x, after_delta_t_y):

        x_point = [start_x, end_x]
        y_point = [start_y, end_y]

        if (slope) == (vector_slope): 
            return True

        else:
            cross_x = (start_x * slope - start_y - OS_pos_x * vector_slope + OS_pos_y) / (slope - vector_slope)
            cross_y = slope * (cross_x - start_x) + start_y

            if OS_pos_x <= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if OS_pos_x <= cross_x <= after_delta_t_x and OS_pos_y <= cross_y <= after_delta_t_y:
                        print(False)
                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x >= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if after_delta_t_x <= cross_x <= OS_pos_x and OS_pos_y <= cross_y <= after_delta_t_y:
                        print(False)
                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x <= after_delta_t_x and OS_pos_y >= after_delta_t_y:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if OS_pos_x <= cross_x <= after_delta_t_x and  after_delta_t_y <= cross_y <= OS_pos_y:
                        print(False)
                        return False
                    else:
                        return True
                else:
                    return True

            else:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if after_delta_t_x <= cross_x <= OS_pos_x and after_delta_t_y <= cross_y <= OS_pos_y:
                        print(False)
                        return False
                    else:
                        return True
                else:
                    return True

    def choose_velocity_vector(self,reachableVel_global_all):
        vector_desired = 0
        for reachable_angel in enumerate(reachableVel_global_all):
            if abs(math.degree(math.atan2(self.goal_y - self.boat_y, self.goal_x - self.boat_x))):
                vector_desired = reachable_angel[3]
        return vector_desired
    
    def cal_distance_goal(self):
        self.distance_goal=math.hypot(self.boat_x-self.goal_x,self.boat_y-self.goal_y)

    def end_check(self):
        self.cal_distance_goal()

        if self.distance_goal<=self.goal_range:
            return True
        else:
            return False

    def rearrange_angle(input_angle):
        if input_angle >= 180:
            output_angle = -180 + abs(input_angle) % 180
        elif input_angle <= -180:
            output_angle = 180 - abs(input_angle) % 180
        else:
            output_angle = input_angle
        return output_angle

    def cal_err_angle(self):
        self.psi_goal = math.degrees(math.atan2(self.goal_y - self.boat_y, self.goal_x - self.boat_x)) - self.psi
        self.error_angle = self.rearrange_angle(self.psi_goal)
        self.psi_desire = self.rearrange_angle(self.psi + self.error_angle)

    def servo_PID(self):
        # P ctrl
        error_angle = self.psi_desire() # deg

        # I ctrl
        self.errSum += (error_angle * 0.1)
        
        if self.errSum > 90:
            self.errSum = 90
        elif self.errSum < -90:
            self.errSum = -90
        else:
            pass

        # D ctrl
        yaw_rate = math.degrees(self.psi) # deg/s


        cp_servo = self.kp_servo * error_angle
        ci_servo = self.ki_servo * self.errSum
        cd_servo = self.kd_servo * -yaw_rate

        servo_pid = -(cp_servo + ci_servo + cd_servo)
        self.servo_control = self.servo_middle + servo_pid

        if self.servo_control > 93+25: #94+24
            self.servo_control = 93+25
        elif self.servo_control < 93-25:
            self.servo_control = 93-25
        else:
            pass

        return self.servo_control
    
    def control_pub(self):
        self.cal_err_angle()
        self.u_servo=self.servo_PID()

        self.servo_pub.publish(int(self.u_servo))
        self.thruster_pub.publish(int(self.u_thruster))

    def print_state(self):
        if self.servo_control > 93+3: # left turn
            turn = "left"
        elif self.servo_control < 93-3: # right turn
            turn = "right"
        else:
            turn = "mid"

        print('-------------------------------------')
        print("distance, thruster : ", self.distance_goal, self.u_thruster)
        print("my xy : ",self.boat_x, self.boat_y)
        print("goal xy : ",self.goal_x, self.goal_y)
        print("psi, desire : ", self.psi, self.psi_desire)
        print("servo : " + turn, round(self.servo_control))
        print('-------------------------------------')

def main():
    rospy.init_node("Autonomous")

    autonomous=Autonomous()

    while not rospy.is_shutdown():
        if autonomous.end_check():
            autonomous.servo_pub.publish(autonomous.servo_middle)
            autonomous.thruster_pub.publish(1500)
            print("FINISHED")
        else:
            autonomous.control_pub()

        autonomous.print_state()

if __name__=="__main__":
    main()