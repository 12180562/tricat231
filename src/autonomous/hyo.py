#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import rospy
import numpy as np

from geometry_msgs.msg import Point
from std_msgs.msg import Float64, UInt16
from sensor_msgs.msg import LaserScan

class Autonomous:
    def __init__(self):
        self.waypoint=rospy.get_param("waypoint")
        self.goal_x=self.waypoint[0][0]
        self.goal_y=self.waypoint[0][1]
        
        self.boat_x,self.boat_y=0
        self.goal_range=rospy.get_param("goal_range")

        self.kp_servo = rospy.get_param("kp_servo")
        self.ki_servo = rospy.get_param("ki_servo")
        self.kd_servo = rospy.get_param("kd_servo")

        # directions
        self.psi = 0  # 자북과 heading의 각도(자북 우측 +, 좌측 -) [degree]
        self.psi_goal = 0  # 현재 선수각으로부터 goal까지 가기 위해 움직여야 할 각도. 선박 기준(-180 ~ 180)
        self.error_angle = 0  # 다음 목표까지 가기 위한 차이각
        self.psi_desire = 0  # 지구고정좌표계 기준 움직여야 할 각도

        # other fix values
        self.servo_middle = int((self.servo_range[0] + self.servo_range[1]) / 2)
        self.thruster_min = rospy.get_param("thruster_min")
        self.thruster_max = rospy.get_param("thruster_max")

        # other variables
        self.distance_to_goal = 0  # 다음 목표까지 남은 거리
        self.u_servo = self.servo_middle
        self.u_thruster = self.thruster_min

        rospy.Subscriber("/heading",Float64,self.heading_callback)
        rospy.Subscriber("/enu_postion",Point,self.boat_position_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        #라이다 콜백

        self.servo_pub=rospy.Publisher("/servo",UInt16)
        self.thruster_pub=rospy.Publisher("/thruster",UInt16)

    def heading_callback(self,msg):
        self.psi=msg.data

    def boat_position_callback(self,msg):
        self.boat_x=msg.x
        self.boat_y=msg.y

    def scan_callback(self, msg):
        self.input_points = []
        phi = msg.angle_min  # 각 점의 각도 계산 위해 계속 누적해갈 각도
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                self.input_points.append(r)
            phi += msg.angle_increment

    #라이다 콜백 추가

    def delete_vector_inside_obstacle(self, reachableVel_global_all, OS):
        # for obstacle detecting, we use some line data like start point, end point, linear equation
        
        static_OB_data = [[[[50.0, 100.0], [100.0, 100.0]], [[100.0, 100.0], [100.0, 50.0]], [[100.0, 50.0], [50.0, 50.0]], [[50.0, 50.0], [50.0, 100.0]]], [[[-50.0, -100.0], [-100.0, -100.0]], [[-100.0, -100.0], [-100.0, -50.0]], [[-100.0, -50.0], [-50.0, -50.0]], [[-50.0, -50.0], [-50.0, -100.0]]]]
        #장애물 리스트 -> 라이다 데이터로 추가해주기

        pA = np.array([OS['Pos_X'], OS['Pos_Y']])
        delta_t = 40 # seconds

        # iteration for all the line data
        for line_data in static_OB_data:
            for i in line_data:
                obstacle_point_x = [i[0][0],i[1][0]] # x끼리, y끼리 묶어서 포인트 셋 만들어 줌 (clear)
                obstacle_point_y = [i[0][1],i[1][1]] # (clear)

                if obstacle_point_x[0] > obstacle_point_x[1]:
                    obstacle_point_x.reverse()
                    obstacle_point_y.reverse()

                if (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) > 0:
                    slope = 9999 # for avoid zero divizion # (clear)

                elif (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) < 0:
                    slope =-9999

                else: #(clear)
                    slope = (obstacle_point_y[1]-obstacle_point_y[0])/(obstacle_point_x[1]-obstacle_point_x[0]) # (clear)
                # if obstacle point have same, x coordinate, it can make error, that is zero devision. so we have to solve this problem
                reachableVel_global_all_after_delta_t = reachableVel_global_all * delta_t # (clear)
                result = []
                #print(slope)

                for i in range(len(reachableVel_global_all_after_delta_t)-1): #(claer)
                    after_delta_t_x = reachableVel_global_all_after_delta_t[i][0]+pA[0]
                    after_delta_t_y = reachableVel_global_all_after_delta_t[i][1]+pA[1]
                    if (pA[0]-after_delta_t_x) == 0:
                        vector_slope = 9999
                    else:
                        vector_slope = (pA[1]-after_delta_t_y)/(pA[0]-after_delta_t_x)

                    #print(vector_slope)
                    # we use get_crosspt function for check the point that is crossed by two line (vector, obstacle)
                    if self.get_crosspt(slope, vector_slope, obstacle_point_x[0], obstacle_point_y[0],obstacle_point_x[1], obstacle_point_y[1], pA[0], pA[1], after_delta_t_x, after_delta_t_y):
                        component = reachableVel_global_all[i]
                        component_list = component.tolist()
                        result.append(component_list)
                    else:
                        pass

                reachableVel_global_all = np.array(result)

        return reachableVel_global_all
            
    def get_crosspt(self, slope, vector_slope, start_x, start_y,end_x, end_y, OS_pos_x, OS_pos_y, after_delta_t_x, after_delta_t_y):

        x_point = [start_x, end_x]
        y_point = [start_y, end_y]

        if (slope) == (vector_slope): # parellel
            return True

        # if two lines are parellel, it can be two situation, firstone is two line is same and can make collision, secondone is just parellel. (no point makes collision)
        # so, we have to make some function that varifiy the type of line, firstone or secondone
        else:
            cross_x = (start_x * slope - start_y - OS_pos_x * vector_slope + OS_pos_y) / (slope - vector_slope)
            cross_y = slope * (cross_x - start_x) + start_y
            #print(f"현재 위치 : ({OS_pos_x},{OS_pos_y})")
            #print(f"미래 위치 : {after_delta_t_x},{after_delta_t_y}")
            #print(f"교점 : {cross_x},{cross_y}")

            if OS_pos_x <= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                #print("first")
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if OS_pos_x <= cross_x <= after_delta_t_x and OS_pos_y <= cross_y <= after_delta_t_y: # vector cross, False (delete vector)
                        print(False)
                        return False
                    else:
                        #print(True)
                        return True
                else:
                    return True

            elif OS_pos_x >= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                #print("second")
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if after_delta_t_x <= cross_x <= OS_pos_x and OS_pos_y <= cross_y <= after_delta_t_y: # vector cross, False (delete vector)
                        print(False)
                        return False
                    else:
                        #print(True)
                        return True
                else:
                    return True

            elif OS_pos_x <= after_delta_t_x and OS_pos_y >= after_delta_t_y:
                #print("third")
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if OS_pos_x <= cross_x <= after_delta_t_x and  after_delta_t_y <= cross_y <= OS_pos_y: # vector cross, False (delete vector)
                        print(False)
                        return False
                    else:
                        #print(True)
                        return True
                else:
                    return True

            else:
                #print("forth")
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if after_delta_t_x <= cross_x <= OS_pos_x and after_delta_t_y <= cross_y <= OS_pos_y: # vector cross, False (delete vector)
                        print(False)
                        return False
                    else:
                        #print(True)
                        return True
                else:
                    return True

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

