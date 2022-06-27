#!/usr/bin/env python2
import tf2_ros
import rospy
from geometry_msgs.msg import TransformStamped, Point, Quaternion

def broadcast():
    rospy.init_node('robot_frame')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    rPose = TransformStamped()
    rPose.header.frame_id = 'camera_pose_frame'
    rPose.child_frame_id = 'robot_pose_frame'
    rPose.transform.translation = Point(0,0,-0.14)
    rPose.transform.rotation = Quaternion(0,0,0,1)

    broadcaster.sendTransform(rPose)
    rospy.spin()

if __name__ == '__main__':
    try:
        broadcast()
    except rospy.ROSInterruptException:
        pass