#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pymap3d as pm
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix

origin = [37.450631,126.6529135,49.451]#[37.450746, 126.6549274, 55.5]
boat = [0, 0, 0]

def gps_fix_callback(msg):
    boat[0], boat[1], boat[2] = enu_convert([msg.latitude, msg.longitude, msg.altitude])

def enu_convert(gnss):
    e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], origin[0], origin[1], origin[2])
    return n, e, u

def main():
    rospy.init_node("gnss_converter", anonymous=True)
    rospy.Subscriber("/ublox_gps/fix", NavSatFix, gps_fix_callback, queue_size=1)
    pub = rospy.Publisher("enu_position", Point, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    enu_position = Point()

    while not rospy.is_shutdown():
        enu_position.x = boat[0]
        enu_position.y = boat[1]

        pub.publish(enu_position)

        rate.sleep()

if __name__ == "__main__":
    main()