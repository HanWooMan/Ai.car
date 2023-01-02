#! /usr/bin/env python

import rospy
from std_msgs.msg import String

rospy.init_node('tutorial')
publisher = rospy.Publisher('/YOLO',String,queue_size = 10)
rate = rospy.Rate(20)

while not rospy.is_shutdown():
    publisher.publish('YOLO')
    rate.sleep()
