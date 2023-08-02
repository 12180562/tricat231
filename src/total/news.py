#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import time
import rospy
import numpy as np

from math import sin, cos, radians, degrees, hypot, atan2

from geometry_msgs.msg import Point
from std_msgs.msg import Float64, UInt16, Bool
from sensor_msgs.msg import Imu
from tricat231_pkg.msg import ObstacleList

from utils import gnss_converter as gc
from utils import static_obstacle_cal as so

class Total_Static:
    def __init__(self):
        #Goal
        self.remained_waypoint = []
        gnss_waypoint = rospy.get_param("waypoints")
        for waypoint in gnss_waypoint:
            n,e,_ = gc.enu_convert(waypoint)
            enu_waypoint = [n, e]
            self.remained_waypoint.append(enu_waypoint)

        self.goal_x = self.remained_waypoint[0][0] # x=0 y=1이 맞음
        self.goal_y = self.remained_waypoint[0][1] 

        self.goal_range = rospy.get_param("goal_range")
        self.distance_goal = 0
        self.psi_desire = 0
        self.target_angle = 0
        self.count = 0
        
        #My Boat
        self.psi = 0
        self.psi_queue = []
        self.filter_queue_size = rospy.get_param("filter_queue_size")
        self.yaw_rate = 0

        self.boat_x = 0 
        self.boat_y = 0
        self.boat_x_queue = []
        self.boat_y_queue = []

        self.servo_range = rospy.get_param("servo_range")
        self.servo_middle = int((self.servo_range[0] + self.servo_range[1]) / 2) 
        self.u_servo = self.servo_middle
        self.u_thruster = int(rospy.get_param("thruster"))

        #PID Control
        self.errSum = 0
        self.kp_servo = rospy.get_param("kp_servo")
        self.kd_servo = rospy.get_param("kd_servo")
        
        #Lidar
        self.obstacles = [] 

        #ROS
        # sub
        self.yaw_rate_sub = rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback, queue_size=1)
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_position_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)

        # pub
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=1)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=1)
        self.finish_pub = rospy.Publisher("/finish_check", Bool, queue_size=1) # YOO 카메라 관련된 네이밍 필요 
        self.finish = False
        
        # rviz pub
        self.psi_pub  = rospy.Publisher("/psi",Float64, queue_size=1)
        self.desire_pub = rospy.Publisher("/psi_desire", Float64, queue_size=1)
        self.end_pub = rospy.Publisher("/end_check", Bool, queue_size=1) # Yoo 도착인지 뭔지 명확한 네이밍 필요
        self.end = False
        
        #Static Obstacle
        self.angle_number = rospy.get_param("angle_number")
        self.detecting_angle = rospy.get_param("detecting_angle")
        self.margin = rospy.get_param("margin")

        self.range = rospy.get_param("so_range")
        self.non_cross_vector_len = 0

    def yaw_rate_callback(self, msg):
        self.yaw_rate = msg.angular_velocity.z 

    def heading_callback(self, msg):
        self.psi = self.moving_avg_filter(self.psi_queue, self.filter_queue_size, msg.data)
        # self.psi = msg.data

    def boat_position_callback(self, msg):
        self.boat_x = self.moving_avg_filter(self.boat_y_queue, self.filter_queue_size, msg.x)
        self.boat_y = self.moving_avg_filter(self.boat_x_queue, self.filter_queue_size, msg.y)
        # self.boat_x = msg.x #x=x
        # self.boat_y = msg.y #y=y

    def obstacle_callback(self, msg):
        self.obstacles = msg.obstacle

    # publish function
    def rviz_publish(self):
        self.psi_pub.publish(self.psi)
        self.desire_pub.publish(self.psi_desire)
        self.end_pub.publish(self.end)

    # senser conection check
    def is_all_connected(self):
        rospy.wait_for_message("/heading", Float64)
        print("\n{:><70}".format("heading_calculator Connected "))
        rospy.wait_for_message("/enu_position", Point)
        print("\n{:><70}".format("gnss_converter Connected "))
        rospy.wait_for_message("/obstacles", ObstacleList)
        print("\n{:><70}".format("lidar_converter Connected "))
        return True
    
    # 이동 평균 필터
    def moving_avg_filter(self, queue, queue_size, input, use_prev=False):         
        if not use_prev:
            if len(queue) >= queue_size:
                queue.pop(0)
            queue.append(input)
        return sum(queue) / float(len(queue))

    def end_check(self):
        self.distance_goal = hypot(self.boat_x-self.goal_x, self.boat_y-self.goal_y)
        return self.distance_goal <= self.goal_range

    def next(self):
        self.count += 1
        if self.count == len(self.remained_waypoint):
            pass
        else:
            self.goal_x = self.remained_waypoint[self.count][0] # x = 0
            self.goal_y = self.remained_waypoint[self.count][1] # y = 1
            
    # Step 1. make detecting vector
    def make_detecting_vector(self):
        detecting_points = np.zeros([self.angle_number+1,3])
        psi = self.psi
        angle_list = [psi]

        for i in range(int(self.angle_number/2)):
            angle_list.append(psi + (i+1)*self.detecting_angle/(self.angle_number/2))
            angle_list.append(psi - (i+1)*self.detecting_angle/(self.angle_number/2))
        
        for j in range(len(angle_list)):
            detecting_points[j][0] = cos(radians(angle_list[j]))
            detecting_points[j][1] = sin(radians(angle_list[j]))
            
            if angle_list[j] > 180:
                detecting_points[j][2] = -180 + abs(angle_list[j]) % 180
            elif angle_list[j] < -180:
                detecting_points[j][2] = 180 - abs(angle_list[j]) % 180
            else:
                detecting_points[j][2] = angle_list[j]

        return detecting_points, psi
                
    # Step 2. delete vector inside obstacle
    def delete_vector_inside_obstacle(self, detecting_points,psi):
        static_OB_data = []
        for i in self.obstacles:
            begin_x = self.boat_x + (-i.begin.x) * cos(radians(psi)) - i.begin.y * sin(radians(psi))
            begin_y = self.boat_y + (-i.begin.x) * sin(radians(psi)) + i.begin.y * cos(radians(psi))
            end_x = self.boat_x + (-i.end.x) * cos(radians(psi)) - i.end.y * sin(radians(psi))
            end_y = self.boat_y + (-i.end.x) * sin(radians(psi)) + i.end.y * cos(radians(psi))
            static_OB_data.extend([begin_x, begin_y, end_x, end_y])

        pA = [self.boat_x, self.boat_y]
        
        non_cross_vector = []
        for i in range(self.angle_number+1):
            tf = []
            for obstacle_number in range(0, len(static_OB_data), 4):     
                oblist = [static_OB_data[obstacle_number],static_OB_data[obstacle_number+1],static_OB_data[obstacle_number+2],static_OB_data[obstacle_number+3]]
                tf.append(so.staticOB_cal(pA[0], pA[1], detecting_points[i][0], detecting_points[i][1], oblist[0], oblist[1], oblist[2], oblist[3], self.range, self.margin).cross_check())

            if True in tf: 
                continue
            else:
                non_cross_vector.append(detecting_points[i][2])

        if len(non_cross_vector) == 0:
            non_cross_vector.append(detecting_points[self.angle_number][2])
            non_cross_vector.append(detecting_points[self.angle_number-1][2])

        self.non_cross_vector_len = int(len(non_cross_vector))
        return non_cross_vector

    # Step3. choose vector
    def vector_choose(self,non_cross_vector):
        minNum = 1000
        vector_desired = 0 
        target_angle = degrees(atan2(self.goal_y - self.boat_y, self.goal_x - self.boat_x)) + 6.5

        #출력
        self.target_angle = target_angle 

        for n in range(len(non_cross_vector)):
            absNum = abs(non_cross_vector[n] - target_angle)

            if absNum >= 180:
                absNum = abs(-180 + abs(absNum) % 180)
            elif absNum <= -180:
                absNum = abs(180 - abs(absNum) % 180)
            else:
                absNum  

            if absNum < minNum:
                minNum = absNum
                vector_desired = non_cross_vector[n]
            else:
                pass

        return vector_desired
    
    # Step4. PID control
    def servo_pid_controller(self):
        psi_desire = self.vector_choose(self.delete_vector_inside_obstacle(self.make_detecting_vector()))

        control_angle = psi_desire - self.psi
        
        # 출력
        self.psi_desire = psi_desire
        
        if control_angle >= 180:
            control_angle = -180 + abs(control_angle) % 180
        elif control_angle <= -180:
            control_angle = 180 - abs(control_angle) % 180
        else:
            control_angle

        cp_servo = self.kp_servo * control_angle
        yaw_rate = degrees(self.yaw_rate)
        cd_servo = self.kd_servo * (-yaw_rate)

        servo_pd = int(-(cp_servo + cd_servo))
        self.u_servo = self.servo_middle + servo_pd

        if self.u_servo > self.servo_range[1]:
            self.u_servo = self.servo_range[1]
        elif self.u_servo < self.servo_range[0]:
            self.u_servo = self.servo_range[0]

        return int(self.u_servo)
    
    def thruster_control(self):
        for i in range(1500, self.u_thruster, 50):
            self.thruster_pub.publish(i)
            time.sleep(0.1)

    def control_publish(self):
        self.servo_pid_controller()
        self.servo_pub.publish(self.u_servo)
        self.thruster_control()
        # self.thruster_pub.publish(self.u_thruster)
    
    def print_state(self):
        print(f"------------------------------------\n \
            distance, thruster : {self.distance_goal}, {self.u_thruster}\n \
            my xy : {self.boat_x}, {self.boat_y}\n \
            goal xy : {self.goal_x}, {self.goal_y}\n \
            psi, desire : {round(self.psi,2)}, {round(self.psi_desire,2)}\n \
            target angle: {round(self.target_angle,4)}\n \
            arriver vector: {self.non_cross_vector_len}\n \
            servo : {self.u_servo}\n \
            count: {self.count}\n")

def main():
    rospy.init_node("Total_Static", anonymous=False)
    rate = rospy.Rate(10) # 10 Hz
    total_static = Total_Static()

    
    while not total_static.is_all_connected():
        rospy.sleep(0.2)
        print("\n{:<>70}".format(" All Connected !"))

    while not rospy.is_shutdown():

        total_static.end = total_static.end_check()
        
        total_static.print_state()

        if total_static.end:
            total_static.next()
            print(f"{total_static.count} arrive")

            if total_static.count == len(total_static.remained_waypoint):
                total_static.servo_pub.publish(total_static.servo_middle)
                total_static.thruster_pub.publish(1500)
                print("-------------Finished---------------")
                break
            else:
                start_time = time.time()
                while time.time() - start_time < 3:
                    total_static.servo_pub.publish(total_static.u_servo)
                    total_static.thruster_pub.publish(1550)
                    print(time.time() - start_time)

        else:
            pass

        total_static.control_publish()
        
        total_static.rviz_publish()
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()