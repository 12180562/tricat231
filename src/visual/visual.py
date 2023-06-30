#!/usr/bin/env python
#-*- coding:utf-8 -*-

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
import math
from collections import deque
import utils.show as sh
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped, Point, Vector3, PoseStamped, PolygonStamped, Point32
from visualization_msgs.msg import Marker, MarkerArray 
from tricat231_pkg.msg import ObstacleList
import utils.gnss_converter as gc
from math import sin, cos, pi
import numpy as np
# from urdf_parser_py.urdf import URDF

class map_rviz:
    def __init__(self):
        self.frame_id = "/map"
        # waypoint
        self.remained_waypoint = []
        gnss_waypoint = rospy.get_param("waypoints")
        for waypoint in gnss_waypoint:
            n,e,_ = gc.enu_convert(waypoint)
            enu_waypoint = [n, e]
            self.remained_waypoint.append(enu_waypoint)
        self.goal_range = rospy.get_param("goal_range")

        rospy.init_node("polygon_array_sample")
        self.pub_waypoint = rospy.Publisher("/waypoint", PolygonStamped, queue_size=10)

    def CirclePolygon(self, num):
        p = PolygonStamped()
        for i in range(100):
            theta = i / 100.0 * 2.0 * pi
            x = self.goal_range * cos(theta) + self.remained_waypoint[num][0]
            y = self.goal_range * sin(theta) + self.remained_waypoint[num][1]
            p.polygon.points.append(Point32(x=x, y=y))
        return p
    
    def publish_waypoint(self):
        num = list(range(0,2))
        waypoint = []
        for i in num:
            waypoint[i] = self.CirclePolygon(i)
        waypoint.header.frame_id = self.frame_id
        waypoint.header.stamp = rospy.Time.now()
        self.pub_waypoint.publish(waypoint)   

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
 
class boat_rviz:
    def __init__(self):
        # self.boat_model = "boat231.urdf"
        self.boat_x = 0.0
        self.boat_y = 0.0
        self.psi = 0
        
        self.boat_trajectory = Path()
        self.goal_trajectory = Path()
        self.previous_position = Point()

        self.frame_id = "/map"
        self.threshold = 0.001
        self.max_poses = 1000

        #sub
        self.enu_pos_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.heading_sub = rospy.Subscriber("/heading", Float64 , self.boat_heading_callback, queue_size=1)
        
        #pub
        self.pub_stamp = rospy.Publisher('/boat_position_rviz', PointStamped, queue_size=10)
        self.boat_heading_rviz_pub = rospy.Publisher('/boat_heading_rviz', MarkerArray, queue_size=10, latch=True)
        self.boat_trajectory_pub = rospy.Publisher('/boat_trajectory_rviz', Path, queue_size=10, latch=True)


    def boat_position_callback(self,msg):
        self.boat_y = msg.x
        self.boat_x = msg.y

    def boat_heading_callback(self,msg):
        self.psi = msg.data

    def heading_rviz(self):
        ids = list(range(1, 100))
        psi_arrow_end_x = 2 * math.cos(math.radians(self.psi)) + self.boat_x
        psi_arrow_end_y = 2 * math.sin(math.radians(self.psi)) + self.boat_y
        psi = sh.arrow_rviz(
            name="psi",
            id=ids.pop(),
            x1=self.boat_x,
            y1=self.boat_y,
            x2=psi_arrow_end_x,
            y2=psi_arrow_end_y,
            color_r=221,
            color_g=119,
            color_b=252,
        )
        psi_txt = sh.text_rviz(name="psi", id=5, text="psi", x=psi_arrow_end_x, y=psi_arrow_end_y)
        heading = sh.marker_array_rviz([psi, psi_txt])

        return heading

    def publish_boat_position(self):
        boat_point = PointStamped()
        boat_point.header.frame_id = self.frame_id 
        boat_point.header.stamp = rospy.Time.now()
        boat_point.point = Vector3(self.boat_x, self.boat_y,0)
        self.pub_stamp.publish(boat_point)

    def publish_heading(self):
        heading = self.heading_rviz()
        self.boat_heading_rviz_pub.publish(heading)

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


def main():
    rospy.init_node('visual')
    boat = boat_rviz()
    obstacle = obstacle_rviz()
    map = map_rviz()

    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        
        boat.publish_heading()
        boat.publish_trajectory()
        boat.publish_boat_position()
        obstacle.publish_obstacle()
        map.publish_waypoint()
        rate.sleep()


if __name__ == '__main__':
    main()