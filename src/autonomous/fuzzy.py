#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import time
import math
import rospy
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64, UInt16
from tricat231_pkg.msg import ObstacleList

from utils import gnss_converter as gc

# Get the current folder path
# If you get an error message, use this: sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
# sys.path.append(os.getcwd())

class Fuzzy:
    # Define valuables
    def __init__(self):
        self.end_time=0
        self.start_time=0
        self.obstacles = []

        self.boat_x, self.boat_y = 0, 0 # Boat coordinate (x,y)
        self.goal_y, self.goal_x, _ = gc.enu_convert(rospy.get_param("autonomous_goal")) # Goal coordinate (x,y)
        self.trajectory = [] # Trajectory that has been moved so far

        self.distance_to_goal = 0
        
        # Goal range to judge finish(radius)
        self.goal_range = rospy.get_param("goal_range")  
        
        # Boat position(heading)
        self.psi = 0.0
        self.psi_goal = 0
        # Boat position(yaw)
        self.yaw_rate = 0

        # LiDAR
        self.angle_min = 0.0 
        self.angle_increment = 0.0
        self.ranges = []
        self.danger_ob = {}

        # FUZZY
        self.fuzzy_servo_control =  0
        self.target_servo_ang = None

        # PID
        self.servo_range = rospy.get_param("servo_range")  # 서보모터 최대/최소값
        self.servo_middle = (self.servo_range[0] + self.servo_range[1]) / 2 # 서보모터 중간값

        self.kp_servo = rospy.get_param("kp_servo")
        self.kd_servo = rospy.get_param("kd_servo")

        self.thruster = rospy.get_param("thruster")

        self.yaw_rate_sub = rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback)
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_position_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)

        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)
        self.clo=0
        # self.print_cnt = 0

    # Callback functions
    def yaw_rate_callback(self, msg):
        self.yaw_rate = msg.angular_velocity.z  # yaw_rate [rad/s]

    def heading_callback(self,msg):
        # filtered_data_list = []
        self.psi=msg.data

        # if len(filtered_data_list) >= 1:
        #     self.psi = msg.data
        #     filtered_data_list.append(msg.data)
        # else:
        #     filtered_data_list.append(msg.data)
        #     filtered_data = (filtered_data_list[2] + filtered_data_list[1] + filtered_data_list[0])/3
        #     self.psi = filtered_data
        #     del filtered_data_list[0]

    def boat_position_callback(self, msg):
        self.boat_y = msg.x
        self.boat_x = msg.y

    def obstacle_callback(self, msg):
        self.obstacles = msg.obstacle
        # print("obstacle_check",self.obstacles)

    def lidar_callback(self, data):
        # print(data.header.seq)
        self.angle_min = data.angle_min
        self.angle_increment = data.angle_increment
        self.ranges = data.ranges  # list

    # Check sensor connection
    # def connection_check(self): 
    #     not_connected = ""  # 아직 연결되지 않은 센서 목록
    #     if self.yaw_rate_sub.get_num_connections() == 0:
    #         not_connected += "IMU\t"
    #     if self.heading_sub.get_num_connections() == 0:
    #         not_connected += "headingCalculator\t"
    #     if self.enu_position_sub.get_num_connections() == 0:
    #         not_connected += "gnssConverter\t"
    #     if self.lidar_sub.get_num_connections() == 0:
    #         not_connected += "lidar\t"

    #     if len(not_connected) == 0:
    #         return True  # All connect
    #     else:
    #         print("\nWaiting >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    #         print(not_connected)
    #         print("\n")
    #         return True # Not all connect

    def cal_distance_goal(self):
        self.distance_goal=math.hypot(self.boat_x-self.goal_x,self.boat_y-self.goal_y)

    def end_check(self):
        self.cal_distance_goal()
        return self.distance_goal <= self.goal_range

    # Calculate goal_psi
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

    # PID conrtol use servo motor
    def servo_pid_controller(self):
        error_angle = self.psi_goal  # P ctrl
        cp_servo = self.kp_servo * error_angle

        yaw_rate = math.degrees(self.yaw_rate)  # D ctrl
        cd_servo = self.kd_servo * (-yaw_rate)

        servo_pd = -(cp_servo + cd_servo)
        u_servo = self.servo_middle + servo_pd

        if u_servo > self.servo_range[1]:
            u_servo = self.servo_range[1]
        elif u_servo < self.servo_range[0]:
            u_servo = self.servo_range[1]

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

    # def print_status(self, is_lpp, u_servo):
    #     if self.print_cnt < 5:
    #         self.print_cnt += 1
    #         return
    #     else:
    #         self.print_cnt = 0

    #     # LPP인지 GPP인지
    #     mode = "Lpp" if is_lpp else "Gpp"
    #     print("-" * 70)
    #     print("")
    #     print("Mode | {}".format(mode))
    #     print("")

    #     # goal이 선수의 어느 쪽에 있는지
    #     psi_goal_dir_str = "[   | * ]" if self.psi_goal > 0 else "[ * |   ]"

    #     # 가장 가까운 점의 위치 및 거리
    #     if len(self.danger_ob) == 0:
    #         print("Dangerous Point | {:>4} deg / {:>2} m".format("None", "None"))
    #     else:
    #         print(
    #             "Dangerous Point | {:>4.2f} deg / {:>2.1f} m".format(self.danger_ob["pi"], self.danger_ob["distance"])
    #         )
    #     print("")

    #     # 어느 쪽으로 움직일 건지
    #     if u_servo < self.servo_middle:
    #         error_angle_dir_str = "( M )"
    #     elif u_servo < self.servo_middle:
    #         error_angle_dir_str = "( R )"
    #     else:
    #         error_angle_dir_str = "( L )"

    #     # 서보 얼마나 움직일지
    #     if u_servo > self.servo_middle:
    #         servo_value_str = "<" * ((self.servo_middle - u_servo) // 10)  # go left
    #     else:
    #         servo_value_str = ">" * ((self.servo_middle - u_servo) // 10)  # go right

    #     print("| {:^9} | {:^9} | {:^10} |".format("heading", "goal", "delta_servo"))
    #     print("{:=^38}".format(""))
    #     print(
    #         "| {:>9.2f} | {:>9.2f} | {:>4d} {:5} |".format(
    #             self.psi, self.psi_goal, self.servo_middle - int(u_servo), error_angle_dir_str
    #         )
    #     )
    #     print("| {:9} | {:^9} | {:^10} |".format("", psi_goal_dir_str, servo_value_str))
    #     print("")

    #     # 남은 거리
    #     print("{:<9} : {:6.2f} m".format("distance", self.distance_to_goal))
    #     print("")
    #     print("-" * 70)

    def print_state(self):
        # if self.u_servo > 93+3: # left turn
        #     turn = "left"
        # elif self.u_servo < 93-3: # right turn
        #     turn = "right"
        # else:
        #     turn = "mid"

        # print('-------------------------------------')
        # print("lpp : ", self.fuzzy_control_avoidance())
        # # print("reminde waypoint : ", self.remained_waypoint)
        # # print("reminde waypoint index : ", self.waypoint_idx)
        # print("distance, thruster : ", self.distance_to_goal, self.thruster)
        # print("my xy : ",self.boat_x, self.boat_y)
        # print("goal xy : ",self.goal_x, self.goal_y)
        # print("psi, desire : ", self.psi, self.psi_goal)
        # print("servo :", self.servo_pid_controller()-93)
        # print("target rule : ",self.fuzzy_servo_control)
        # print("servo : " + turn, round(self.u_servo))
        # print('-------------------------------------')
        # print(f"------------------------------------\nlpp : {self.fuzzy_control_avoidance()}\ndistance, thruster : {self.distance_goal}, {self.thruster}\nmy xy : {self.boat_x}, {self.boat_y}\ngoal xy : {self.goal_x}, {self.goal_y}\npsi, desire : {self.psi}, {self.psi_desire}\nservo : {self.servo_pid_controller()-93}\ntarget rule : {self.fuzzy_servo_control}")
        print(f"------------------------------------\n \
              lpp : {self.fuzzy_control_avoidance()}\n \
              distance, thruster : {self.distance_goal}, {self.thruster}\n \
              my xy : {self.boat_x}, {self.boat_y}\n \
              goal xy : {self.goal_x}, {self.goal_y}\n \
              psi, desire : {self.psi}, {self.psi_desire}\n \
              servo : {self.servo_pid_controller()-93}\n \
              range : {self.clo}\n \
              target rule : {self.fuzzy_servo_control}")

def main():
    rospy.init_node("fuzzy_ctrl", anonymous=False)
    rate = rospy.Rate(10) # 10 Hz
    fuzz = Fuzzy()
    fuzz.fuzzy()  
    
    # while not fuzz.connection_check():
    #     rospy.sleep(0.2)
    # print("\n<<<<<<<<<<<<<<<<<<< All Connected !")

    while not rospy.is_shutdown():
        fuzz.start_time=fuzz.end_time
        fuzz.cal_err_angle()
        if fuzz.end_check():
            # fuzz.thruster_pub.publish(1500)  # 정지
            print(">>>>>>>>>>>>>> Finished <<<<<<<<<<<<<<")
            return
        else:
            fuzz.trajectory.append([fuzz.boat_x, fuzz.boat_y])
            is_lpp = fuzz.fuzzy_control_avoidance()
            if is_lpp:
                u_servo = fuzz.servo_middle + fuzz.fuzzy_servo_control
            else:
                u_servo = fuzz.servo_pid_controller()

            fuzz.servo_pub.publish(int(u_servo))
            fuzz.thruster_pub.publish(int(fuzz.thruster))

        # fuzz.print_status(is_lpp, u_servo)
        fuzz.print_state()  
        fuzz.end_time=time.time()
        print("time : ",fuzz.end_time-fuzz.start_time)
        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    main()