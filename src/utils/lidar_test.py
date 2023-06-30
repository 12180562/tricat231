#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys 
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
from math import sqrt
import numpy as np
from sensor_msgs.msg import LaserScan
from utils.lidar_calc import*
from tricat231_pkg.msg import Obstacle, ObstacleList

class Lidar_Converter:
    def __init__(self):

        self.boat_x, self.boat_y = 0, 0  # 현재 보트 위치

        # sub, pub
        rospy.Subscriber("/scan", LaserScan, self.lidar_raw_callback, queue_size=1)
        self.obstacle_pub = rospy.Publisher("/obstacles", ObstacleList, queue_size=10)
        
        # params
        # self.max_gap_in_set = rospy.get_param("max_gap_in_set") # 한 그룹으로 묶을 포인트 간 거리 ( x 0.1 meters )
        # self.point_set_size = rospy.get_param("point_set_size") # 한 그룹의 점의 개수
        # self.max_dist_to_ps_line = rospy.get_param("max_dist_to_ps_line") # 그룹에서부터 점까지 최대 거리 ( x 0.1 meters )
        # self.min_wall_length = rospy.get_param("min_wall_length") # 벽이라고 인정할 최소 길이 ( x 0.1 meters )
        # self.wall_particle_length = rospy.get_param("wall_particle_length") # 벽 분할할 길이 ( x 0.1 meters )
        # self.min_input_points_size = rospy.get_param("min_input_points_size") # 최소 스캐닝 포인트 개수

        self.max_gap_in_set = 1 # 한 그룹으로 묶을 포인트 간 거리 ( x 0.1 meters )
        self.point_set_size = 3 # 한 그룹의 점의 개수
        self.max_dist_to_ps_line = 1 # 그룹에서부터 점까지 최대 거리 ( x 0.1 meters )
        self.min_wall_length = 1 # 벽이라고 인정할 최소 길이 ( x 0.1 meters )
        self.wall_particle_length = 2 # 벽 분할할 길이 ( x 0.1 meters )
        self.min_input_points_size = 3 # 최소 스캐닝 포인트 개수
        
        # scanning / converted data
        self.input_points = []  # 라이다에서 받은 모든 점들을 (x, y) 형태로 저장 (J: raw data 집어넣는 건가 ????)
        self.point_sets_list = []  # point_set들의 리스트 [ps1, ps2, ...]
        self.obstacles = []  # 최종적으로 합쳐지고 나눠인 set들 저장 (wall + buoy)
        self.buoy_particle = []

    def lidar_raw_callback(self, msg):
        """Subscribe lidar scanning data
        Notes:
            * phi
                * angle in radians
                * range: -3.14 (to left) ~ 3.14 (to right), 0 = forward
            * convert polar coordinate system -> cartesian's
            * Origin is LiDAR (boat), not ENU origin
        """
        # initialize all data lists
        self.input_points = []
        self.point_sets_list = []
        self.obstacles = []

        # save range data
        phi = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                p = Point.polar_to_cartesian(r, phi)
                self.input_points.append(p)
            phi += msg.angle_increment

        # # subscriber와 publisher의 sink를 맞추기 위해 이곳에서 모두 진행 (뭘까...)
        self.process_points()
        self.publish_obstacles()

    def process_points(self):
        """Clustering process"""
        if len(self.input_points) < self.min_input_points_size:
            return # J: 뭘 리턴 하는 거지 ???

        self.group_points()
        for ps in self.point_sets_list:
            self.split_group(ps)
        self.classify_groups()

    def group_points(self):
        """Group adjacent raw scanning points
        Notes:
            "del point_set"
                파이썬은 변수 자체가 포인터 역할을 하므로,
                다른 그룹을 만들고자 단순히 이전 point_set의 인스턴스 변수들을 초기화하면
                이전에 append 되었던 point_set들도 모두 동일한 값으로 바뀜
                따라서 메모리 확보를 위해 아예 변수 지우고 다시 선언함
        """
        point_set = PointSet()
        point_set.append_point(self.input_points[0])  # 중복으로 들어가게 될 것임. 마지막에 제거할 것
        for p in self.input_points:
            if p.dist_btw_points(point_set.end) > self.max_gap_in_set:
                if point_set.set_size > self.point_set_size:
                    self.point_sets_list.append(point_set)  # set of point groups

                del point_set  # delete previous group instance
                point_set = PointSet()  # new group
                point_set.append_point(p)
            else:
                point_set.append_point(p)
        self.point_sets_list.append(point_set)  # 마지막 그룹까지 추가

        # 중복으로 들어간 첫 번째 점 제거
        self.point_sets_list[0].begin = self.point_sets_list[0].point_set[1]
        self.point_sets_list[0].set_size -= 1
        del self.point_sets_list[0].point_set[0]

    def split_group(self, ps):
        """Split group(point set) into smaller groups
        Args:
            ps (lidar_list_calc.PointSet): querying point set(group)
        """
        max_distance = 0  # 그룹의 첫점 ~ 끝점 이은 직선으로부터 가장 멀리 떨어진 점까지의 거리
        split_idx = 0  # max_distance를 가지는 점의 그룹 내 인덱스
        point_idx = 0  # 탐색하고 있는 점의 인덱스

        # find the farthest point from group
        for p in ps.point_set:
            dist_to_ps_line = ps.dist_to_point(p)  # distance from line to point
            # print(p.x, "라인까지 거리", dist_to_ps_line)
            if dist_to_ps_line > max_distance:
                max_distance = dist_to_ps_line
                split_idx = point_idx
            point_idx += 1

        # split groups
        if max_distance > self.max_dist_to_ps_line:
            # if two splitted groups would be too small, don't split this
            if split_idx < self.point_set_size or (ps.set_size - split_idx) < self.point_set_size:
                return

            ps1 = PointSet()
            ps1.input_point_set(ps.point_set[:split_idx])
            ps2 = PointSet()
            ps2.input_point_set(ps.point_set[split_idx:])

            self.point_sets_list.insert(self.point_sets_list.index(ps), ps1)
            self.point_sets_list.insert(self.point_sets_list.index(ps), ps2)

            del self.point_sets_list[self.point_sets_list.index(ps)]  # 나눠서 저장한 뒤 원본 그룹 삭제

            # split groups recursively
            self.split_group(ps1)
            self.split_group(ps2)

    def classify_groups(self):
        """Classify groups(point sets) as Walls or Buoys
        If length of group is long, classify as wall and split it into small groups.
        Append it to final clustering result list after splitting in "split_wall()" function.
        If not, just append it to final clustering list ("obstacles")
        """
        for ps in self.point_sets_list:
            if ps.dist_begin_to_end() > self.min_wall_length:
                self.split_wall(ps)
            else:
                self.clusturing_buoy(ps)

    
    def clusturing_buoy(self, ps):
        
        buoy_particle = PointSet()
        buoy_particle.append_point(ps.begin)

        min = 999

        for p in ps.point_set:

            res = sqrt(pow(p.y - self.boat_y, 2.0) + pow(p.x - self.boat_x, 2.0))

            if res  < min:
                min = res
                closed_point = [p.x,p.y] # 가장 가까운 점

        # print(buoy_particle)

        # self.obstacles.append(buoy_particle)
        # buoy_particle.append_point(p)

        for p in ps.point_set:

            line_inclination = ps.inclination()

            line_y = line_inclination*closed_point[0] + closed_point[1]
            line_n = line_inclination*closed_point[0]+line_y


            begin_point_angle = ps.begin_point_angle_calc()
            end_point_angle = ps.end_point_angle_calc()


            begin_r =  line_n/(np.sin(begin_point_angle) - line_inclination*np.cos(begin_point_angle))
            end_r =  line_n/(np.sin(end_point_angle) - line_inclination*np.cos(end_point_angle))

            new_begin_point = [begin_r*np.cos(begin_point_angle),begin_r*np.sin(begin_point_angle)]
            new_end_point = [end_r*np.cos(end_point_angle),end_r*np.sin(end_point_angle)]



            new_buoy_list = [new_begin_point,new_end_point]

            # print(new_buoy_list)

            self.buoy_particle.append(new_buoy_list)
            self.obstacles.append(buoy_particle)

    def split_wall(self, ps):
        """Split long groups into short ones
        Notes:
            "del wall_particle"
                -> group_points() 함수 참고
        """
        wall_particle = PointSet()
        wall_particle.append_point(ps.begin)
        for p in ps.point_set:
            if p.dist_btw_points(wall_particle.begin) > self.wall_particle_length:
                self.obstacles.append(wall_particle)
                del wall_particle
                wall_particle = PointSet()
            wall_particle.append_point(p)
        self.obstacles.append(wall_particle)  # last group


    def publish_obstacles(self):
        """Publish clustering results in (x, y) coordinate format"""
        ob_list = ObstacleList()

        for ob in self.obstacles:
            obstacle = Obstacle()
            obstacle.begin.x = ob.begin.x
            obstacle.begin.y = ob.begin.y
            obstacle.end.x = ob.end.x
            obstacle.end.y = ob.end.y
            ob_list.obstacle.append(obstacle)

        print(ob_list)

        self.obstacle_pub.publish(ob_list)


def main():
    rospy.init_node("LidarConverter", anonymous=False)
    lidar_converter = Lidar_Converter()
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        rate.sleep()
        


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass