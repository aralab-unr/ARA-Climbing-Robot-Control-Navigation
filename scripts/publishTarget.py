#!/usr/bin/env python3
from geometry_msgs.msg import Point
import rospy

if __name__ == '__main__':
    rospy.init_node("publishTarget")
    vis_pub = rospy.Publisher("Target", Point, queue_size=10)
    vis_pub.publish(Point(0.7,0,1.317))