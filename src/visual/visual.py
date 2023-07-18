#!/usr/bin/env python
#-*- coding:utf-8 -*-

#----- 파일 경로 찾는 부분----------------------------------------------------------------------------------------------------------------------#
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
#--------------------------------------------------------------------------------------------------------------------------------------------#

import rospy
import math
from collections import deque # obstacle 부분에서 deque 자료구조 쓸 때 사용하는 거
import utils.show as sh 
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PointStamped, Point, Vector3, PoseStamped, PolygonStamped, Point32, Polygon
from visualization_msgs.msg import MarkerArray
from tricat231_pkg.msg import ObstacleList
import utils.gnss_converter as gc # gnss converter인데 이거 안쓰고 total,total_hyo에서 받아오고 싶은데
import numpy as np
from math import sin, cos, pi
# from urdf_parser_py.urdf import URDF

# ----- 경기장----------------------------------------------------------------------------------------------------------------------#
class map_rviz:
    def __init__(self):

        # Waypoint
        # Get waypoint, goal range param
        gnss_waypoint = rospy.get_param("waypoints")
        goal_range = rospy.get_param("goal_range")

        # 분수대 오른쪽1(구글 어스 위도, 경도): 37.4483338, 126.6537511 // -254.9589107162451, 74.11673378428729
        # 분수대 오른쪽2(구글 어스 위도, 경도): 37.4487297, 126.6539784 // -211.01897805798822, 94.22933843885015
        # 분수대 왼쪽1(구글 어스 위도, 경도): 37.4483338, 126.6537511  // -254.95887062301057, 74.1167221783096
        self.bsd = sh.Square_polygon(-254.9589107162451, 74.11673378428729)
        self.pub_bsd = rospy.Publisher("/bsd", PolygonStamped, queue_size=10)

        # Waypoint
        self.remained_polygon = []
        for waypoint in gnss_waypoint:
            n,e,_ = gc.enu_convert(waypoint)
            self.remained_polygon.append(sh.Circle_polygon([n, e], goal_range))
        
        self.end_sub = rospy.Subscriber("/end_check", Bool, self.end_callback, queue_size=10)
        self.end = False
        self.pub_waypoint1 = rospy.Publisher("/waypoint1", PolygonStamped, queue_size=10)
        self.pub_waypoint2 = rospy.Publisher("/waypoint2", PolygonStamped, queue_size=10)
        self.pub_waypoint3 = rospy.Publisher("/waypoint3", PolygonStamped, queue_size=10)
    
    def publish_map(self):
        self.pub_bsd.publish(self.bsd)
        cnt = 0
        if self.end:
            cnt += 1
        if cnt < 1:
            self.pub_waypoint1.publish(self.remained_polygon[0])
            self.pub_waypoint2.publish(self.remained_polygon[1])
            self.pub_waypoint3.publish(self.remained_polygon[2])
        elif cnt == 1:
            self.pub_waypoint2.publish(self.remained_polygon[1])
            self.pub_waypoint3.publish(self.remained_polygon[2])
        elif cnt == 2:
            self.pub_waypoint3.publish(self.remained_polygon[2])
        else:
            print("Finish")

    def end_callback(self, msg):
        self.end = msg.data
    
#--------------------------------------------------------------------------------------------------------------------------------------------#

# ----- Lidar 시각화----------------------------------------------------------------------------------------------------------------------#
class obstacle_rviz:
    def __init__(self):
        self.threshold = 1000
        self.ids = deque(list(range(1,self.threshold)))
        self.obstacles = []
        # sub
        rospy.Subscriber('/obstacles',ObstacleList, self.obstacle_callback)
        # pub
        self.rviz_pub = rospy.Publisher("/obstacles_rviz", MarkerArray, queue_size=10)
        
    def obstacle_callback(self,msg):
        self.obstacles = msg.obstacle

    def osbtacle_rviz(self):
        obstacle = []
        for ob in self.obstacles:
            obstacle.append([ob.begin.x, ob.begin.y])
            obstacle.append([ob.end.x, ob.end.y])
        ids = self.ids.pop()
        self.ids.append(ids)
        obstacle = sh.linelist_rviz(
            name="obstacle", id=ids, lines=obstacle, color_r=255, color_g=255, color_b=0, scale=0.1
        )
        return obstacle

    def publish_obstacle(self):
        obstacle = self.osbtacle_rviz()
        markers = sh.marker_array_rviz([obstacle])
        self.rviz_pub.publish(markers)
#--------------------------------------------------------------------------------------------------------------------------------------------#

# ----- 배 관련 시각화(position, heading, trajectory)--------------------------------------------------------------------------------------------#
class boat_rviz:
    def __init__(self):
        # self.boat_model = "boat231.urdf"
        self.boat_x = 0.0
        self.boat_y = 0.0
        self.psi = 0 # 배가 바라보는 각도(자북에서 부터) 
        # 자북? 나침반이 가리키는 방향(Imu) / 진북? 언제나 변하지 않는 방향(gps) = > 두 개를 잡는 방법 생각
        self.psi_desire = 0 # 배가 쫓아야 하는 각도
        
        self.boat_trajectory = Path()
        self.goal_trajectory = Path()
        self.previous_position = Point()

        self.frame_id = "map"
        self.threshold = 0.001
        self.max_poses = 1000

        #sub
        self.enu_pos_sub = rospy.Subscriber("/boat_position", Point, self.boat_position_callback, queue_size=1)
        self.psi_sub = rospy.Subscriber("/psi", Float64 , self.psi_callback, queue_size=1)
        self.psi_desire_sub = rospy.Subscriber("/psi_desire", Float64 , self.psi_desire_callback, queue_size=1)
        
        #pub
        self.pub_stamp = rospy.Publisher('/boat_position_rviz', PointStamped, queue_size=1)
        self.heading_pub = rospy.Publisher('/heading_rviz', MarkerArray, queue_size=1, latch=True)
        self.choose_vec_pub = rospy.Publisher('/choose_vec', MarkerArray, queue_size=1, latch=True)
        self.boat_trajectory_pub = rospy.Publisher('/boat_trajectory_rviz', Path, queue_size=1, latch=True)

    def boat_position_callback(self,msg):
        self.boat_x = msg.x
        self.boat_y = msg.y

    def psi_callback(self, msg):
        self.psi = msg.data

    def psi_desire_callback(self,msg):
        self.psi_desire = msg.data

    def choose_vec_rviz(self):
        length = 1
        ids = list(range(1, 100))
        choose_vec_arrow_end_x = length * math.cos(math.radians(self.psi)) + self.boat_x
        choose_vec_arrow_end_y = length * math.sin(math.radians(self.psi)) + self.boat_y
        choose_vec = sh.arrow_rviz(
            name="psi",
            id=ids.pop(),
            x1=self.boat_x,
            y1=self.boat_y,
            x2=choose_vec_arrow_end_x,
            y2=choose_vec_arrow_end_y,
            color_r=0,
            color_g=255,
            color_b=0,
        )
        choose_vec_txt = sh.text_rviz(name="choose_vec", id=5, text="choose_vec", x=choose_vec_arrow_end_x, y=choose_vec_arrow_end_y)
        choose_vec_m = sh.marker_array_rviz([choose_vec, choose_vec_txt])
        return choose_vec_m
    
    def heading_rviz(self):
        length = 1
        ids = list(range(1, 100))
        heading_arrow_end_x = length * math.cos(math.radians(self.psi_desire)) + self.boat_x
        heading_arrow_end_y = length * math.sin(math.radians(self.psi_desire)) + self.boat_y
        heading = sh.arrow_rviz(
            name="psi",
            id=ids.pop(),
            x1=self.boat_x,
            y1=self.boat_y,
            x2=heading_arrow_end_x,
            y2=heading_arrow_end_y,
            color_r=221,
            color_g=119,
            color_b=252,
        )
        heading_txt = sh.text_rviz(name="heading", id=6, text="heading", x=heading_arrow_end_x, y=heading_arrow_end_y)
        heading_m = sh.marker_array_rviz([heading, heading_txt])
        return heading_m

    def publish_boat_position(self):
        boat_point = PointStamped()
        boat_point.header.frame_id = self.frame_id 
        boat_point.header.stamp = rospy.Time.now()
        boat_point.point = Vector3(self.boat_x, self.boat_y,0)
        self.pub_stamp.publish(boat_point)

    def publish_heading(self):
        heading = self.heading_rviz()
        choose_vec = self.choose_vec_rviz()
        self.heading_pub.publish(heading)
        self.choose_vec_pub.publish(choose_vec)

    def publish_trajectory(self):
        if((abs(self.previous_position.x - self.boat_x) > self.threshold) or (abs(self.previous_position.y - self.boat_y) > self.threshold)):
            rospy.logdebug('Error')
            self.boat_trajectory.header.stamp = rospy.Time.now()
            self.boat_trajectory.header.frame_id = self.frame_id
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = self.boat_x
            pose_stamped.pose.position.y = self.boat_y

            if len(self.boat_trajectory.poses) < self.max_poses:
                self.boat_trajectory.poses.append(pose_stamped)
            else:
                rospy.logdebug("Max number of poses reached, erasing oldest pose")
                self.boat_trajectory.poses = self.boat_trajectory.poses[1:]
                self.boat_trajectory.poses.append(pose_stamped)
            self.previous_position = pose_stamped.pose.position
            self.boat_trajectory_pub.publish(self.boat_trajectory)
#--------------------------------------------------------------------------------------------------------------------------------------------#

def main():
    rospy.init_node('visual')
    boat = boat_rviz()
    obstacle = obstacle_rviz()
    map = map_rviz()

    rate = rospy.Rate(10) # 10Hz
    try:
        while not rospy.is_shutdown():
            
            map.publish_map()
            boat.publish_boat_position()
            boat.publish_heading()
            boat.publish_trajectory()
            obstacle.publish_obstacle()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()