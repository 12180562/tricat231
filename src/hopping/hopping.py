#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import math
import rospy

from geometry_msgs.msg import Point
from std_msgs.msg import Float64, UInt16

from utils import gnss_converter as gc
# from utils import heading_converter as hc

class Hopping:
    def __init__(self):
        self.remained_waypoint = []
        gnss_waypoint = rospy.get_param("waypoints")

        for waypoint in gnss_waypoint:
            n,e,_ = gc.enu_convert(waypoint)
            enu_waypoint = [n, e]
            self.remained_waypoint.append(enu_waypoint)

        self.goal_x = self.remained_waypoint[0][0]
        self.goal_y = self.remained_waypoint[0][1]

        # directions
        self.psi = 0  # 자북과 heading의 각도(자북 우측 +, 좌측 -) [degree]
        self.psi_desire = 0  # 지구고정좌표계 기준 움직여야 할 각도

        self.boat_x=0
        self.boat_y=0

        self.errSum=0

        self.distance_goal = 0  # 다음 목표까지 남은 거리

        self.goal_range=rospy.get_param("goal_range")

        self.kp_servo = rospy.get_param("kp_servo")
        self.ki_servo = rospy.get_param("ki_servo")
        self.kd_servo = rospy.get_param("kd_servo")

        self.kp_distance = rospy.get_param("kp_distance") 
        self.ki_distance = rospy.get_param("ki_distance")  
        self.kd_distance = rospy.get_param("kd_distance")

        self.servo_range = rospy.get_param("servo_range")
        self.servo_middle = int((self.servo_range[0] + self.servo_range[1]) / 2) 
        self.thruster_min = rospy.get_param("thruster_min")
        self.thruster_max = rospy.get_param("thruster_max")

        self.u_servo = self.servo_middle
        self.u_thruster = self.thruster_min

        self.servo_control=0

        self.heading_sub = rospy.Subscriber("/heading",Float64,self.heading_callback) # 재윤: connection_check에 변수 필요해서 변수 설정함
        self.enu_position_sub = rospy.Subscriber("/enu_position",Point,self.boat_position_callback) # 재윤: connection_check에 변수 필요해서 변수 설정함

        self.servo_pub = rospy.Publisher("/servo",UInt16)
        self.thruster_pub = rospy.Publisher("/thruster",UInt16)

        self.cal_distance_goal()
        self.cal_err_angle()

    def heading_callback(self,msg):
        self.psi=msg.data

    def boat_position_callback(self,msg):
        self.boat_y=msg.x
        self.boat_x=msg.y

    # Check sensor connection - 수정 필요
    # def connection_check(self): 
    #     not_connected = ""  # 아직 연결되지 않은 센서 목록
    #     if self.heading_sub.get_num_connections() == 0:
    #         not_connected += "headingCalculator\t"
    #     if self.enu_position_sub.get_num_connections() == 0:
    #         not_connected += "gnssConverter\t"

    #     if len(not_connected) == 0:
    #         return True  # All connect
    #     else:
    #         print("\nWaiting >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    #         print(not_connected)
    #         print("\n")
    #         return False  # Not all connect

    def cal_distance_goal(self):
        self.distance_goal=math.hypot(self.boat_x-self.goal_x,self.boat_y-self.goal_y)
    
    def next(self):
        del self.remained_waypoint[0]

        self.goal_x=self.remained_waypoint[0][0]
        self.goal_y=self.remained_waypoint[0][1]

        # del self.remained_waypoint[self.waypoint_idx]
        # self.waypoint_idx+=1
        
        # # if len(self.gnss_waypoint) + 1 == self.waypoint_idx:
        # #     return
        
        # # 위에 부분에서 고쳐지면 이 부분도 문제가 있는거 아닌가...
        # self.goal_x=self.remained_waypoint[self.waypoint_idx][0]
        # self.goal_y=self.remained_waypoint[self.waypoint_idx][1]

    def end_check(self):
        self.cal_distance_goal()
        return self.distance_goal <= self.goal_range

    def rearrange_angle(self, input_angle):
        if input_angle >= 180:
            output_angle = -180 + abs(input_angle) % 180
        elif input_angle <= -180:
            output_angle = 180 - abs(input_angle) % 180
        else:
            output_angle = input_angle
        return output_angle

    def cal_err_angle(self):
        self.psi_desire = math.degrees(math.atan2(self.goal_y - self.boat_y, self.goal_x - self.boat_x))-self.psi

        return self.rearrange_angle(self.psi_desire)

    def servo_PID(self):
        # P ctrl
        error_angle = self.cal_err_angle() # deg

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
    
    def distance_PID(self):
        cp_distance = self.kp_distance * self.distance_goal
        cd_distance = -self.kd_distance * self.distance_goal / 0.1  # dt = rate

        u_distance = cp_distance + cd_distance
        u_thruster = self.thruster_min + u_distance
        
        if u_thruster > self.thruster_max:
            u_thruster = self.thruster_max
        elif u_thruster < self.thruster_min:
            u_thruster = self.thruster_min

        return int(u_thruster)
    
    def control_pub(self):
        # self.cal_err_angle()
        self.u_servo=self.servo_PID()

        # self.cal_distance_goal()
        self.u_thruster=self.distance_PID()

        self.servo_pub.publish(int(self.u_servo))
        self.thruster_pub.publish(int(self.u_thruster))

    def print_state(self):
        if self.servo_control > 93+3: # left turn
            turn = "left"
        elif self.servo_control < 93-3: # right turn
            turn = "right"
        else:
            turn = "mid"

        # print('-------------------------------------\n')
        # print(f"reminde waypoint : {self.remained_waypoint}\n")
        # # print("reminde waypoint index : ", self.waypoint_idx)
        # print(f"distance, thruster : {self.distance_goal}, {self.u_thruster}\n")
        # print(f"my xy : {self.boat_x}, {self.boat_y}\n")
        # print(f"goal xy : {self.goal_x}, {self.goal_y}\n")
        # print(f"psi, desire : {self.psi}, {self.psi_desire}\n")
        # print(f"servo : \n" + turn, round(self.servo_control))
        # print('-------------------------------------')
        print(f"------------------------------------\nreminde waypoint : {self.remained_waypoint}\ndistance, thruster : {self.distance_goal}, {self.u_thruster}\nmy xy : {self.boat_x}, {self.boat_y}\ngoal xy : {self.goal_x}, {self.goal_y}\npsi, desire : {self.psi}, {self.psi_desire}\nservo : {self.u_servo-93}")

def main():
    rospy.init_node("HoppingTour")

    hopping=Hopping()

    while not rospy.is_shutdown():
        if len(hopping.remained_waypoint)==0:
            hopping.servo_pub.publish(hopping.servo_middle)
            hopping.thruster_pub.publish(1500)
            print("FINISHED")
            
        else:
            if hopping.end_check():
                hopping.next()
                print("NOW END, GO NEXT")
            hopping.control_pub()

        hopping.print_state()

if __name__=="__main__":
    main()