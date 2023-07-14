#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import math
import rospy
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from geometry_msgs.msg import Point
from std_msgs.msg import Float64, UInt16, Bool
from sensor_msgs.msg import Imu, LaserScan
from tricat231_pkg.msg import ObstacleList

from utils import gnss_converter as gc

class Total:
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
        
        self.psi_desire = 0

        #My Boat
        self.psi = 0
        self.yaw_rate=0

        self.boat_x = 0
        self.boat_y = 0

        self.servo_range = rospy.get_param("servo_range")
        self.servo_middle = int((self.servo_range[0] + self.servo_range[1]) / 2) 
        self.u_servo = self.servo_middle
        
        self.u_thruster = rospy.get_param("thruster")

        #PID Control
        self.errSum=0
        self.kp_servo = rospy.get_param("kp_servo")
        self.ki_servo = rospy.get_param("ki_servo")
        self.kd_servo = rospy.get_param("kd_servo")
        
        #Lidar
        self.obstacles = []

        self.angle_min = 0.0 
        self.angle_increment = 0.0
        self.ranges = []
        self.danger_ob = {}

        #ROS
        self.yaw_rate_sub = rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback)
        self.heading_sub = rospy.Subscriber("/heading",Float64,self.heading_callback)
        self.enu_position_sub = rospy.Subscriber("/enu_position",Point,self.boat_position_callback)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)

        self.servo_pub = rospy.Publisher("/servo",UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster",UInt16, queue_size=0)
        self.end_pub = rospy.Publisher("/end_check",Bool, queue_size=10)

        #Initializing
        self.cal_distance_goal()
        self.cal_err_angle()

        #Fuzzy
        self.fuzzy_servo_control =  0
        self.target_servo_ang = None
        self.clo=0
    
    def yaw_rate_callback(self, msg):
        self.yaw_rate = msg.angular_velocity.z 

    def heading_callback(self,msg):
            self.psi=msg.data

    def boat_position_callback(self,msg):
            self.boat_y=msg.x
            self.boat_x=msg.y

    def lidar_callback(self, data):
        self.angle_min = data.angle_min
        self.angle_increment = data.angle_increment
        self.ranges = data.ranges

    def obstacle_callback(self, msg):
        self.obstacles = msg.obstacle

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
        self.psi_desire = math.degrees(math.atan2(self.goal_y - self.boat_y, self.goal_x - self.boat_x))-self.psi
        if self.psi_desire >= 180:
            output_angle = -180 + abs(self.psi_desire) % 180
        elif self.psi_desire <= -180:
            output_angle = 180 - abs(self.psi_desire) % 180
        else:
            output_angle = self.psi_desire
        return output_angle
    
    def servo_pid_controller(self):
        error_angle = self.cal_err_angle()  # P ctrl
        cp_servo = self.kp_servo * error_angle

        yaw_rate = math.degrees(self.yaw_rate)  # D ctrl
        cd_servo = self.kd_servo * (-yaw_rate)

        servo_pd = -(cp_servo + cd_servo)
        u_servo = self.servo_middle + servo_pd

        if u_servo > self.servo_range[1]:
            u_servo = self.servo_range[1]
        elif u_servo < self.servo_range[0]:
            u_servo = self.servo_range[0]

        return int(u_servo)

    def fuzzy(self):
        # Define input variables
        distance = ctrl.Antecedent(np.arange(0, 4, 0.1), "distance")
        angle = ctrl.Antecedent(np.arange(-70, 70, 1), "angle")
        
        # Define output variable
        target_servo = ctrl.Consequent(np.arange(-30, 30, 1), "target_servo")
        
        # Define membership functions for input variables
        ## distance
        distance["ED"] = fuzz.trapmf(distance.universe, [0, 0, 0.5, 1.5])
        distance["D"] = fuzz.trimf(distance.universe, [0.5, 1.5, 2.5])
        distance["W"] = fuzz.trimf(distance.universe, [1.5, 2.5, 3.5])
        distance["B"] = fuzz.trapmf(distance.universe, [2.5, 3.5, 4, 4])
        ## angle
        angle["NL"] = fuzz.trapmf(angle.universe, [-70, -50, -40, -30]) # Negative Large
        angle["NM"] = fuzz.trapmf(angle.universe, [-40, -30, -20, -10]) # Negative Medium
        angle["NS"] = fuzz.trimf(angle.universe, [-25, 0, 1])           # Negative Small
        angle["PS"] = fuzz.trimf(angle.universe, [0, 1, 25])            # Positive Small
        angle["PM"] = fuzz.trapmf(angle.universe, [10, 20, 30, 40])     # Positive Medium
        angle["PL"] = fuzz.trapmf(angle.universe, [30, 40, 50, 70])     # Positive Large

        # Define membership functions for output variable
        ## servo motor
        target_servo["RE"] = fuzz.trimf(target_servo.universe, [-30, -27, -18]) # Rigtht Extra
        target_servo["RL"] = fuzz.trimf(target_servo.universe, [-18, -16, -13]) # Right Large
        target_servo["RM"] = fuzz.trimf(target_servo.universe, [-18, -13, -7])  # Right Medium
        target_servo["RS"] = fuzz.trimf(target_servo.universe, [-12, -7, 0])    # Right Small
        target_servo["N"] = fuzz.trimf(target_servo.universe, [0, 0, 0])
        target_servo["LS"] = fuzz.trimf(target_servo.universe, [0, 7, 12])      # Left Small
        target_servo["LM"] = fuzz.trimf(target_servo.universe, [7, 13, 18])     # Left Medium
        target_servo["LL"] = fuzz.trimf(target_servo.universe, [13, 16, 18])    # Left Large
        target_servo["LE"] = fuzz.trimf(target_servo.universe, [18, 27, 30])    # Left Extra

        # Define rules
        rule_ED_NL = ctrl.Rule(distance["ED"] & angle["NL"], target_servo["RM"])
        rule_ED_NM = ctrl.Rule(distance["ED"] & angle["NM"], target_servo["RL"])
        rule_ED_NS = ctrl.Rule(distance["ED"] & angle["NS"], target_servo["RE"])
        rule_ED_PS = ctrl.Rule(distance["ED"] & angle["PS"], target_servo["LE"])
        rule_ED_PM = ctrl.Rule(distance["ED"] & angle["PM"], target_servo["LL"])
        rule_ED_PL = ctrl.Rule(distance["ED"] & angle["PL"], target_servo["LM"])

        rule_D_NL = ctrl.Rule(distance["D"] & angle["NL"], target_servo["RS"])
        rule_D_NM = ctrl.Rule(distance["D"] & angle["NM"], target_servo["RM"])
        rule_D_NS = ctrl.Rule(distance["D"] & angle["NS"], target_servo["RL"])
        rule_D_PS = ctrl.Rule(distance["D"] & angle["PS"], target_servo["LL"])
        rule_D_PM = ctrl.Rule(distance["D"] & angle["PM"], target_servo["LM"])
        rule_D_PL = ctrl.Rule(distance["D"] & angle["PL"], target_servo["LS"])

        rule_W_NL = ctrl.Rule(distance["W"] & angle["NL"], target_servo["N"])
        rule_W_NM = ctrl.Rule(distance["W"] & angle["NM"], target_servo["RS"])
        rule_W_NS = ctrl.Rule(distance["W"] & angle["NS"], target_servo["RM"])
        rule_W_PS = ctrl.Rule(distance["W"] & angle["PS"], target_servo["LM"])
        rule_W_PM = ctrl.Rule(distance["W"] & angle["PM"], target_servo["LS"])
        rule_W_PL = ctrl.Rule(distance["W"] & angle["PL"], target_servo["N"])

        rule_B_NL = ctrl.Rule(distance["B"] & angle["NL"], target_servo["N"])
        rule_B_NM = ctrl.Rule(distance["B"] & angle["NM"], target_servo["N"])
        rule_B_NS = ctrl.Rule(distance["B"] & angle["NS"], target_servo["RS"])
        rule_B_PS = ctrl.Rule(distance["B"] & angle["PS"], target_servo["LS"])
        rule_B_PM = ctrl.Rule(distance["B"] & angle["PM"], target_servo["N"])
        rule_B_PL = ctrl.Rule(distance["B"] & angle["PL"], target_servo["N"])

        # Base class to contain a Fuzzy Control System
        target_servo_ctrl = ctrl.ControlSystem(
            [rule_ED_NL, rule_ED_NM, rule_ED_NS, rule_ED_PL, rule_ED_PM, rule_ED_PS,
             rule_D_NL, rule_D_NM, rule_D_NS, rule_D_PL, rule_D_PM, rule_D_PS,
             rule_W_NL, rule_W_NM, rule_W_NS, rule_W_PL, rule_W_PM, rule_W_PS,
             rule_B_NL, rule_B_NM, rule_B_NS, rule_B_PL, rule_B_PM, rule_B_PS])
        
        self.target_servo_ang = ctrl.ControlSystemSimulation(target_servo_ctrl)
    
    # Avoidance algorithm using fuzzy theory
    def fuzzy_control_avoidance(self):
        '''
        return
            True: 장애물이 탐지 범위(각, 거리) 안에 있음, 지역경로계획(LPP : Local Path-Planning)
            False: 그 외, 전역경로계획(GPP :Global Path-Planning)
        '''
        
        # 라이다 콜백 함수와 겹쳐 현재 탐색 중인 데이터가 아닌 방금 들어온 데이터를 쓸 위험이 있음. 따라서 탐색 중인 데이터를 따로 제작
        self.danger_ob = {}

        # opt 각도 범위 내 값들만 골라냄 (라이다 raw 기준(z축 inverted) -70도는 )
        start_idx = int((math.radians(110)) / (self.angle_increment + 0.00001))
        end_idx = int((math.radians(250)) / (self.angle_increment + 0.00001))
        ranges = self.ranges[start_idx : (end_idx + 1)]

        if ranges == [] or min(ranges) == float("inf"):
            return False

        closest_distance = min(ranges)  # 가장 가까운 장애물까지 거리

        self.clo=closest_distance

        idx = ranges.index(closest_distance) + start_idx  # 가장 가까운 장애물의 인덱스
        # pi = -math.degrees(angle_min + angle_increment * idx) + 180
        pi = math.degrees(self.angle_min + self.angle_increment * idx)

        # If angle is over 180 deg or under -180 deg, rearrange it
        
        # lidar는 후방이 0 -> 왼쪽으로 돌아 전방이 180 -> 후방이 360
        # pi = self.rearrange_angle(pi)

        if (0.3 <= closest_distance <= 2.8) and (-70 <= pi <= 70): # 장애물이 배로부터 2.8m 이하로 있고, 각도가 좌우 70도 이내일 때
            self.target_servo_ang.input["distance"] = float(closest_distance)
            self.target_servo_ang.input["angle"] = float(pi)
            self.target_servo_ang.compute()
            self.fuzzy_servo_control = int(self.target_servo_ang.output["target_servo"])

            self.danger_ob["distance"] = closest_distance
            self.danger_ob["idx"] = idx
            self.danger_ob["pi"] = pi

            return True
        else:
            return False

def main():
    rospy.init_node("Total")
    rate = rospy.Rate(10) # 10 Hz
    total = Total()

    total.fuzzy() 
    count = 0

    while not rospy.is_shutdown():

        is_lpp = total.fuzzy_control_avoidance()
        if is_lpp:
            total.u_servo = total.servo_middle + total.fuzzy_servo_control
        else:
            total.u_servo = total.servo_pid_controller()

        if total.end_check():
            total.end_pub.publish(True)
            total.next()
            count+=1
            print("arrive")
            # 3초 지연 코드 필요
        else:
            total.end_pub.publish(False)
            pass
 

        if count == 0:
            total.servo_pub.publish(total.u_servo)
        
        if count == 1:
            print("11111111111")
            # 카메라 켜기
            total.servo_pub.publish(total.u_servo)
            # total.thruster_pub.publish(int(total.u_thruster))
        if count == 2:
            print("2222222222222")
            # 카메라 끄기
            total.servo_pub.publish(total.u_servo)
            # total.thruster_pub.publish(int(total.u_thruster))
        if count == 3:
            print("33333333333333333")
            total.servo_pub.publish(total.u_servo)
            # total.thruster_pub.publish(1500)
            print("-------------Finished---------------")

        print(f"------------------------------------\n \
              lpp : {total.fuzzy_control_avoidance()}\n \
              distance, thruster : {total.distance_goal}, {total.u_thruster}\n \
              my xy : {total.boat_x}, {total.boat_y}\n \
              goal xy : {total.goal_x}, {total.goal_y}\n \
              psi, desire : {total.psi}, {total.psi_desire}\n \
              servo : {total.servo_pid_controller()-93}\n \
              range : {total.clo}\n \
              target rule : {total.fuzzy_servo_control}")
        
        rate.sleep()

    rospy.spin()

if __name__=="__main__":
    main()