#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""Subscribe 2D laser scanner(LiDAR) raw scanning data and cluster them

Notes:
    * Launch file: /tricat221/launch/sensor_test.launch의 <!-- LiDAR --> 부분 주석 해제 후 사용
    * Steps for clustering
        (a) group points
        (b) split groups
        (c) clasify Wall vs Buoy
        (d) split walls
    * Description of Paramters (/tricat221/params/lidar_params)
        * controller:
            bool 형태로, trackbar를 이용해 각 파라미터를 수정하며 실시간으로 결과를 볼 것인지 결정
        * min_input_points_size:
            LiDAR의 scanning data 개수가 일정 개수 이상이어야 계산을 시작하고, 아니라면 클러스터링 수행 및 publish 하지 않음
            scanning data가 없다면 무시한다는 의미.
        * max_gap_in_set: 
            Step (a)에서 '이 점이 근방의 점과 얼마나 떨어졌는가'를 계산하고, "max_gap_in_set" 이상이 되면 서로 다른 그룹으로 분류
            >>>>> 멀리 떨어진 점까지도 한 그룹으로 묶고 싶다면 이 값을 높게 설정
        * point_set_size:
            한 그룹의 점의 개수. 
            Step (a)에서는 '지금까지 모아온 점들의 개수'가 "point_set_size"보다 커야만 그룹들의 리스트에 추가. 그보다 적은 개수의 점만 있다면 무시.
            Step (b)에서는 '이 그룹을 둘로 분리할 때, 나눠진 두 그룹의 점의 개수'가 "point_set_size"보다 작으면 그룹 분리 안함.
            >>>>> 아주 작은 그룹도 유지하고 싶다면 이 값을 작게 설정
        * max_dist_to_ps_line:
            Step (b)에서 '이 점이 지금 속해있는 그룹으로부터 얼마나 떨어져 있는가'를 계산하고,
            "max_dist_to_ps_line" 이상이면 그 지점으로부터 두 개의 그룹으로 분리.
            >>>>> 구불구불한 그룹도 하나로 만들고 싶다면 이 값을 크게 설정
        * min_wall_length:
            Step (c)에서 '이 그룹의 길이'를 계산하고 "min_wall_length"보다 크면 부표가 아니라 벽으로 분류해 쪼개기 시작.
            >>>>> 벽을 많이 쪼개고 싶다면 이 값을 작게 설정
        * wall_particle_length:
            Step (d)에서 '그룹 시작에서 이 점까지 거리'가 "wall_particle_length" 이상이면 이 지점에서 벽을 쪼갬
            >>>>> 벽을 잘게 쪼개고 싶다면 이 값을 작게 설정
"""
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import os
import sys
from math import pow, sqrt
import cv2
import rospy

from sensor_msgs.msg import LaserScan
from tricat231_pkg.msg import Obstacle, ObstacleList
from visualization_msgs.msg import MarkerArray

import gnss_converter as gc
import show as visual
from lidar_calc import Point, PointSet

class Lidar_Converter:
    def __init__(self):
        self.boat_x, self.boat_y = 0,0  # 현재 보트 위치
        self.goal_x, self.goal_y, _ = gc.enu_convert(rospy.get_param("autonomous_goal"))  # 목표점 위치

        # on/off
        self.controller = rospy.get_param("controller")
        # params
        self.min_input_points_size = rospy.get_param("min_input_points_size") # 최소 스캐닝 포인트 개수
        self.max_gap_in_set = rospy.get_param("max_gap_in_set") # 한 그룹으로 묶을 포인트 간 거리 ( x 0.1 meters ) 점 간 거리를 의미함
        self.point_set_size = rospy.get_param("point_set_size") # 한 그룹의 점의 개수
        self.max_dist_to_ps_line = rospy.get_param("max_dist_to_ps_line") # 그룹에서부터 점까지 최대 거리 ( x 0.1 meters ) 투영(projection)에서 사용되는 parameter
        self.min_wall_length = rospy.get_param("min_wall_length") # 벽이라고 인정할 최소 길이 ( x 0.1 meters )
        self.wall_particle_length = rospy.get_param("wall_particle_length") # 벽 분할할 길이 ( x 0.1 meters )

        # scanning / converted data
        self.input_points = []  # 라이다에서 받은 모든 점들을 (x, y) 형태로 저장
        self.point_sets_list = []  # point_set들의 리스트 [ps1, ps2, ...]
        self.buoy_particle = [] # 부표로 판단된 장애물들을 저장
        self.obstacles = []  # 최종적으로 합쳐지고 나눠인 set들 저장 (wall + buoy)

        # trackbar
        if self.controller:
            cv2.namedWindow("controller")
            cv2.createTrackbar(
                "max_gap_in_set", "controller", rospy.get_param("max_gap_in_set"), 5, lambda x: x
            )  # X 0.1 meter
            cv2.createTrackbar("point_set_size", "controller", rospy.get_param("point_set_size"), 30, lambda x: x)
            cv2.createTrackbar(
                "max_dist_to_ps_line", "controller", rospy.get_param("max_dist_to_ps_line"), 5, lambda x: x
            )  # X 0.1 meter
            cv2.createTrackbar(
                "min_wall_length", "controller", rospy.get_param("min_wall_length"), 50, lambda x: x
            )  # X 0.1 meter
            cv2.createTrackbar(
                "wall_particle_length", "controller", rospy.get_param("wall_particle_length"), 50, lambda x: x
            )  # X 0.1 meter

        # sub, pub
        rospy.Subscriber("/scan", LaserScan, self.lidar_raw_callback, queue_size=1) 
        self.obstacle_pub = rospy.Publisher("/obstacles", ObstacleList, queue_size=10) # 클러스터링 완료한 장애물 publish 형태
        #begin: 
        #   x: -2.7549397092515244
        #   y: 0.07644232912367258
        #   z: 0.0
        # end: 
        #   x: -2.7520000934600737
        #   y: -2.2520949986649115e-07
        #   z: 0.0

        # 기본 출력형태는 위와 같으며 아래와 같이 원하는 값을 for문을 통해 추출할 수 있다.

        # begin_x
        # -2.7549397092515244
        # begin_y
        # 0.07644232912367258
        # end_x
        # -2.7520000934600737
        # end_y
        # -2.2520949986649115e-07

        self.rviz_pub = rospy.Publisher("/rviz_visual", MarkerArray, queue_size=10)

    def lidar_raw_callback(self, msg): # 초기 raw_data를 받는 callback 함수 부분이다
        """Subscribe lidar scanning data
        Notes:
            * phi
                * angle in radians
                * range: -3.14 (to left) ~ 3.14 (to right), 0 = forward
            * convert polar coordinate system -> cartesian's
            * Origin is LiDAR (boat), not ENU origin
        """
        # initialize all data lists
        self.input_points = [] # 초기 라이다 데이터는 r, phi형태로 들어오는데 이를 좌표형태로 만들어서 넣는 리스트이다.
        self.point_sets_list = []
        self.obstacles = []

        # save range data r, phi형태로 들어오는데 이를 좌표형태로 만들어서 넣는 리스트이다. 나와있는 angle_min 등등은 초기 라이다 msg에 있는 것들이다.
        phi = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                p = Point.polar_to_cartesian(r, phi)
                self.input_points.append(p)

            phi += msg.angle_increment

        # subscriber와 publisher의 sink를 맞추기 위해 이곳에서 모두 진행
        self.get_trackbar_pos()
        self.process_points()
        self.publish_obstacles()
        self.publish_rviz()

    def get_trackbar_pos(self):
        """get trackbar positions and set each values"""
        if self.controller:
            self.max_gap_in_set = cv2.getTrackbarPos("max_gap_in_set", "controller") * 0.1
            self.point_set_size = cv2.getTrackbarPos("point_set_size", "controller")
            self.max_dist_to_ps_line = cv2.getTrackbarPos("max_dist_to_ps_line", "controller") * 0.1
            self.min_wall_length = cv2.getTrackbarPos("min_wall_length", "controller") * 0.1
            self.wall_particle_length = cv2.getTrackbarPos("wall_particle_length", "controller") * 0.1

    def process_points(self):
        """Clustering process"""
        if len(self.input_points) < self.min_input_points_size:  # 넣은 점들 리스트의 개수 < 최소 리스트 개수(param). 즉 스캐닝 포인트 개수
            return 

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
            if p.dist_btw_points(point_set.end) > self.max_gap_in_set: # 장애물들까지의 거리 > 한 그룹으로 묶을 포인트 간 거리(param). 즉 임계값 기준점을 잡는다
                if point_set.set_size > self.point_set_size: # 넣은 그룹내의 점의 개수 > 지정한 그룹내의 점의 개수(param). 즉 집합으로 묶을 수 있는 점의 최소 개수를 설정
                    self.point_sets_list.append(point_set)  # set of point groups 즉 몇개의 그룹을 넣었는지를 또 새로운 리스트(points_sets_list)에 추가

                del point_set  # delete previous group instance
                point_set = PointSet()  # new group
                point_set.append_point(p)
            else:
                point_set.append_point(p)
        self.point_sets_list.append(point_set)  # 마지막 그룹까지 추가

        # 중복으로 들어간 첫 번째 점 제거
        # self.point_sets_list[0].begin = self.point_sets_list[0].point_set[1]
        # self.point_sets_list[0].set_size -= 1
        # del self.point_sets_list[0].point_set[0]

    def split_group(self, ps):
        """Split group(point set) into smaller groups
        Args:
            ps (PointSet): querying point set(group)
        """
        max_distance = 0  # 그룹의 첫점 ~ 끝점 이은 직선으로부터 가장 멀리 떨어진 점까지의 거리
        split_idx = 0  # max_distance를 가지는 점의 그룹 내 인덱스
        point_idx = 0  # 탐색하고 있는 점의 인덱스

        # find the farthest point from group
        for p in ps.point_set:
            dist_to_ps_line = ps.dist_to_point(p)  # distance from line to point 즉 특정 점과 이 PointSet까지의 거리
            # print(p.x, "라인까지 거리", dist_to_ps_line)
            if dist_to_ps_line > max_distance: # 특정 점과 이 PointSet까지의 거리 > 그룹의 첫점 ~ 끝점 이은 직선으로부터 가장 멀리 떨어진 점까지의 거리
                max_distance = dist_to_ps_line
                split_idx = point_idx
            point_idx += 1

        # split groups
        if max_distance > self.max_dist_to_ps_line: # 그룹의 첫점 ~ 끝점 이은 직선으로부터 가장 멀리 떨어진 점까지의 거리 > 그룹에서부터 점까지 최대 거리(param)
            # if two splitted groups would be too small, don't split this
            if split_idx < self.point_set_size or (ps.set_size - split_idx) < self.point_set_size: # 범위를 벗어나면 한 개의 그룹으로 생각해서 classify groups로 넘어간다.
                return
            
            # 2개의 그룹으로 나누어서 각각 새로운 그룹으로 분류를 한 이후 기존의 그룹은 삭제를 하는 과정을 거친다

            ps1 = PointSet()
            ps1.input_point_set(ps.point_set[:split_idx])
            ps2 = PointSet()
            ps2.input_point_set(ps.point_set[split_idx:])

            # print("현재 ps의 인덱스", (self.point_sets_list).index(ps))
            # print("나누기 이전 psl 크기", len(self.point_sets_list))

            self.point_sets_list.insert(self.point_sets_list.index(ps), ps1)
            self.point_sets_list.insert(self.point_sets_list.index(ps), ps2)
            del self.point_sets_list[self.point_sets_list.index(ps)]  # 나눠서 저장한 뒤 원본 그룹 삭

            # split groups을 다시 한 번 수행한다.
            self.split_group(ps1)
            self.split_group(ps2)

    def classify_groups(self):
        """Classify groups(point sets) as Walls or Buoys
        If length of group is long, classify as wall and split it into small groups.
        Append it to final clustering result list after splitting in "split_wall()" function.
        If not, just append it to final clustering list ("obstacles")
        """
        for ps in self.point_sets_list:
            if ps.dist_begin_to_end() > self.min_wall_length: # points_sets_list의 길이 즉 장애물의 길이  >  벽이라고 인정할 최소 길이
                self.split_wall(ps)
                # self.obstacles.append(ps)
            else:
                self.clusturing_buoy(ps)
                # self.obstacles.append(ps)

    def split_wall(self, ps):
        """Split long groups into short ones
        Notes:
            "del wall_particle"
                -> group_points() 함수 참고
        """
        wall_particle = PointSet()
        wall_particle.append_point(ps.begin)
        for p in ps.point_set:
            if p.dist_btw_points(wall_particle.begin) > self.wall_particle_length: # 벽의 길이 > 벽 분할할 길이. 즉 한 개의 쭉 이어진 벽으로 설정하기 보다는 분할을 하는 과정이다.
                self.obstacles.append(wall_particle)
                del wall_particle
                wall_particle = PointSet()
            wall_particle.append_point(p)

        self.obstacles.append(wall_particle)  # last group

    def clusturing_buoy(self, ps):
        buoy_particle = PointSet()

        min = 999

        for p in ps.point_set: # 가장 가까운 점을 찾는 구문
            # print("구 시작점",p.x)
            # print("구 끝점",p.y)

            buoy_particle.append_point(p)
            
            res = sqrt(pow(p.y - self.boat_y, 2.0) + pow(p.x - self.boat_x, 2.0))

            if res  < min:
                min = res
                closed_point = [p.x,p.y] # 가장 가까운 점

        # for p in ps.point_set:
            
        #     if p%2 == 0:
        #         print(ps.point_set.begin[p].x)
        #     else:
        #         print(ps.point_set.end[p].x)

        line_inclination = (ps.point_set[-1].y - ps.point_set[0].y) / (ps.point_set[-1].x - ps.point_set[0].x+0.00000000000000001) # 장애물의 시작점과 끝점의 기울기를 구한다.

        new_line_y = -1*line_inclination*closed_point[0] + closed_point[1]  # ex y=-x+2 면 2
        # print("새로운 선의 y값line_y",new_line_y)

        begin_point_inclination = (ps.point_set[0].y)/(ps.point_set[0].x) #시작점의 기울기
        end_point_inclination = (ps.point_set[-1].y)/(ps.point_set[-1].x) #끝점의 기울기

        # print("시작점의 기울기",begin_point_inclination)
        # print("끝점의 기울기",end_point_inclination)

        new_begin_point_x = ((-new_line_y)/(line_inclination-begin_point_inclination)) #교차 시작점의 x
        new_begin_point_y = ((line_inclination)*(-new_line_y)/(line_inclination-begin_point_inclination) + new_line_y) #교차 시작점의 y

        new_end_point_x = ((-new_line_y)/(line_inclination-end_point_inclination)) # 교차 끝점의 x
        new_end_point_y = ((line_inclination)*(-new_line_y)/(line_inclination-end_point_inclination) + new_line_y) #교차 끝점의 y

        # print(buoy_particle.begin.x)
        buoy_particle.begin.x = new_begin_point_x
        buoy_particle.begin.y = new_begin_point_y
        buoy_particle.end.x = new_end_point_x
        buoy_particle.end.y = new_end_point_y

        # print(buoy_particle.begin.x)
        self.obstacles.append(buoy_particle)

    def publish_obstacles(self):
        """Publish clustering results in (x, y) coordinate format"""
        ob_list = ObstacleList()

        for ob in self.obstacles: # buoy + wall의 장애물 obstacles 들을 전달 msg 타입에 맞춰서 ( ex) Point.begin.x) ob_list에 넣는다
            obstacle = Obstacle()
            obstacle.begin.x = -ob.begin.y
            obstacle.begin.y = ob.begin.x
            obstacle.end.x = -ob.end.y
            obstacle.end.y = ob.end.x
            ob_list.obstacle.append(obstacle)

        self.obstacle_pub.publish(ob_list)

    def publish_rviz(self):
        ids = list(range(0, 100))

        input_points = []
        for p in self.input_points:
            input_points.append([p.x, p.y])
        input_points = visual.points_rviz(name="input_points", id=ids.pop(), points=input_points, color_r=255)

        filtered_points = []  # after delete too small groups after "group_points"
        for ps in self.point_sets_list:
            for p in ps.point_set:
                filtered_points.append([p.x, p.y])
        filtered_points = visual.points_rviz(
            name="filtered_points", id=ids.pop(), points=filtered_points, color_r=235, color_g=128, color_b=52)

        point_set = []  # after "split_group", all groups(point sets)
        for ps in self.point_sets_list:
            point_set.append([ps.begin.x, ps.begin.y])
            point_set.append([ps.end.x, ps.end.y])
        point_set = visual.linelist_rviz(
            name="point_set", id=ids.pop(), lines=point_set, color_r=55, color_g=158, color_b=54, scale=0.1)

        obstacle = []  # after "split_wall", final clustering results
        for ob in self.obstacles:
            obstacle.append([ob.begin.x, ob.begin.y])
            obstacle.append([ob.end.x, ob.end.y])
        obstacle = visual.linelist_rviz(
            name="obstacle", id=ids.pop(), lines=obstacle, color_r=10, color_g=81, color_b=204, scale=0.1)

        all_markers = visual.marker_array_rviz([input_points, filtered_points, point_set, obstacle])
        self.rviz_pub.publish(all_markers)

def main():
    rospy.init_node("LidarConverter", anonymous=False)
    Lidar_Converter()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()