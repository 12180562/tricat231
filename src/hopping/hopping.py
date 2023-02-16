#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import rospy

from geometry_msgs.msg import Point
from std_msgs.msg import Float64, UInt16

class Hopping:
    def __init__(self):
        self.waypoint_idx=1

        self.waypoint=rospy.get_param("waypoints")
        self.remained_waypoints={}
        self.goal_x=self.remained_waypoints[self.waypoint_idx][0]
        self.goal_y=self.remained_waypoints[self.waypoint_idx][1]

        self.boat_x,self.boat_y=0
        self.goal_range=rospy.get_param("goal_range")

        self.kp_servo = rospy.get_param("kp_servo")
        self.ki_servo = rospy.get_param("ki_servo")
        self.kd_servo = rospy.get_param("kd_servo")

        self.kp_distance = rospy.get_param("kp_distance") 
        self.ki_distance = rospy.get_param("ki_distance")  
        self.kd_distance = rospy.get_param("kd_distance")

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
        
        self.servo_pub=rospy.Publisher("/servo",UInt16)
        self.thruster_pub=rospy.Publisher("/thruster",UInt16)

    def heading_callback(self,msg):
        self.psi=msg.data

    def boat_position_callback(self,msg):
        self.boat_x=msg.x
        self.boat_y=msg.y

    def cal_distance_goal(self):
        self.distance_goal=math.hypot(self.boat_x-self.goal_x,self.boat_y-self.goal_y)

    def next(self):
        self.waypoint_idx+=1
        
        self.goal_x=self.remained_waypoints[self.waypoint_idx][0]
        self.goal_y=self.remained_waypoints[self.waypoint_idx][1]

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
    
    def distance_PID(self):
        cp_distance = self.kp_distance * self.distance_to_goal
        cd_distance = -self.kd_distance * self.distance_to_goal / 0.1  # dt = rate

        u_distance = cp_distance + cd_distance
        u_thruster = self.thruster_min + u_distance
        
        if u_thruster > self.thruster_max:
            u_thruster = self.thruster_max
        elif u_thruster < self.thruster_min:
            u_thruster = self.thruster_min

        return int(u_thruster)
    
    def control_pub(self):
        self.cal_err_angle()
        self.u_servo=self.servo_PID()

        self.cal_distance_goal()
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

        print('-------------------------------------')
        print("distance, thruster : ", self.distance_goal, self.u_thruster)
        print("my xy : ",self.boat_x, self.boat_y)
        print("goal xy : ",self.goal_x, self.goal_y)
        print("psi, desire : ", self.psi, self.psi_desire)
        print("servo : " + turn, round(self.servo_control))
        print('-------------------------------------')

def main():
    rospy.init_node("HoppingTour")

    hopping=Hopping()

    while not rospy.is_shutdown():
        if len(hopping.remained_waypoints)==0:
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
