#!/usr/bin/env python3
import tf2_ros
import rospy
from geometry_msgs.msg import TransformStamped, Point, Quaternion
from nav_msgs.msg import Odometry

def broadcast():
    
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rPos = TransformStamped()
    waiting = True
    
    while waiting:
        try:
            rPos = tfBuffer.lookup_transform('camera_odom_frame','robot_pose_frame', rospy.Time())
            waiting = False
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
    # rPos.transform.translation.z = rPos.transform.translation.z - 0.15 
    rPose = TransformStamped()
    rPose.header.frame_id = 'camera_odom_frame'
    rPose.child_frame_id = 'robot_odom_frame'
    rPose.transform.translation = rPos.transform.translation
    rPose.transform.rotation = Quaternion(0,0,0,1)

    broadcaster.sendTransform(rPose)

if __name__ == '__main__':
   rospy.init_node('robot_odom_frame')
   broadcast()
   rospy.spin()