#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import UInt16

def main():
    rospy.init_node("Total")
    rate = rospy.Rate(10) # 10 Hz
    servo_pub = rospy.Publisher("/servo",UInt16, queue_size=0)
    while not rospy.is_shutdown():
        servo_pub.publish(93)
        for i in range(78, 113):
            servo_pub.publish(i)
        rate.sleep()
    
    rospy.spin()



if __name__=="__main__":
    main()