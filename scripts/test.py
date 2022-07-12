#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray

def test():
    pub = rospy.Publisher('wheelAndSteer', Float32MultiArray, queue_size=1)
    rospy.init_node('Controller')
    controller = Float32MultiArray()
    r = rospy.Rate(10)

    controller.data.append(1500)
    controller.data.append(1500)
    controller.data.append(1650)
    controller.data.append(1550)
    pub.publish(controller)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass