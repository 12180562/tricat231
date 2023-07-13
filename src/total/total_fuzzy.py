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
from std_msgs.msg import Float64, UInt16
from sensor_msgs.msg import Imu, LaserScan
from tricat231_pkg.msg import ObstacleList

from utils import gnss_converter as gc

class Total_Fuzzy:
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
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)

        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)

        #Initializing
        self.cal_distance_goal()
        self.cal_err_angle()

        #Fuzzy
        self.fuzzy_servo_control =  0
        self.target_servo_ang = None
        self.clo=0
    
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
        rospy.wait_for_message("/scan", LaserScan)
        print("\n{:><70}".format("LiDAR Connected "))

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
        self.psi_desire = math.degrees(math.atan2(self.goal_y - self.boat_y, self.goal_x - self.boat_x))-self.psi
        if self.psi_desire >= 180:
            output_angle = -180 + abs(self.psi_desire) % 180
        elif self.psi_desire <= -180:
            output_angle = 180 - abs(self.psi_desire) % 180
        else:
            output_angle = self.psi_desire
        return output_angle
    
    def servo_pid_controller(self):
        error_angle = self.cal_err_angle()
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
    
    def fuzzy_control_avoidance(self):
        self.danger_ob = {}

        start_idx = int((math.radians(110)) / (self.angle_increment + 0.00001))
        end_idx = int((math.radians(250)) / (self.angle_increment + 0.00001))
        ranges = self.ranges[start_idx : (end_idx + 1)]

        if ranges == [] or min(ranges) == float("inf"):
            return False

        closest_distance = min(ranges)

        self.clo=closest_distance

        idx = ranges.index(closest_distance) + start_idx
        pi = math.degrees(self.angle_min + self.angle_increment * idx)

        if (0.3 <= closest_distance <= 2.8) and (-70 <= pi <= 70):
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
        
    def print_state(self):
        print(f"------------------------------------\n \
            lpp : {self.fuzzy_control_avoidance()}\n \
            distance, thruster : {self.distance_goal}, {self.u_thruster}\n \
            my xy : {self.boat_x}, {self.boat_y}\n \
            goal xy : {self.goal_x}, {self.goal_y}\n \
            psi, desire : {self.psi}, {self.psi_desire}\n \
            servo : {self.servo_pid_controller()-93}\n \
            range : {self.clo}\n \
            target rule : {self.fuzzy_servo_control}")
        
def main():
    rospy.init_node("Total_Fuzzy", anonymous=False)
    rate = rospy.Rate(10) # 10 Hz

    total_fuzzy = Total_Fuzzy()
    total_fuzzy.fuzzy() 

    count = 0

    while not total_fuzzy.is_all_connected():
        rospy.sleep(0.2)
        print("\n{:<>70}".format(" All Connected !"))

    while not rospy.is_shutdown():
        total_fuzzy.print_state()

        is_lpp = total_fuzzy.fuzzy_control_avoidance()
        if is_lpp:
            total_fuzzy.u_servo = total_fuzzy.servo_middle + total_fuzzy.fuzzy_servo_control
        else:
            total_fuzzy.u_servo = total_fuzzy.servo_pid_controller()

        if total_fuzzy.end_check():
            total_fuzzy.next()
            count+=1
            print("arrive")
            rospy.sleep(3)
        else:
            pass

        if count == 0:
            total_fuzzy.servo_pub.publish(total_fuzzy.u_servo)
        
        if count == 1:
            print("11111111111")
            # 카메라 켜기
            total_fuzzy.servo_pub.publish(total_fuzzy.u_servo)
            total_fuzzy.thruster_pub.publish(total_fuzzy.u_thruster)
        if count == 2:
            print("2222222222222")
            # 카메라 끄기
            total_fuzzy.servo_pub.publish(total_fuzzy.u_servo)
            total_fuzzy.thruster_pub.publish(total_fuzzy.u_thruster)
        if count == 3:
            print("33333333333333333")
            total_fuzzy.servo_pub.publish(total_fuzzy.servo_middle)
            total_fuzzy.thruster_pub.publish(1500)
            print("-------------Finished---------------")
        
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()