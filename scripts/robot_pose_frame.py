#!/usr/bin/env python3
import tf2_ros
import rospy
from geometry_msgs.msg import TransformStamped, Point, Quaternion
from nav_msgs.msg import Odometry

def broadcast():
    
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    rPose = TransformStamped()
    rPose.header.frame_id = 'camera_pose_frame'
    rPose.child_frame_id = 'robot_pose_frame'
    rPose.transform.translation = Point(0,0,-0.19)
    rPose.transform.rotation = Quaternion(0,0,0,1)

    broadcaster.sendTransform(rPose)

if __name__ == '__main__':
    rospy.init_node('robot_pose_frame')
    broadcast()
    rospy.spin()
