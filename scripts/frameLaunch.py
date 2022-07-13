#!/usr/bin/env python3
import roslaunch
import rospy
from dynamic_reconfigure.msg import BoolParameter
from nav_msgs.msg import Odometry


if __name__ == '__main__':
    rospy.init_node('frameLaunch')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    realsense = roslaunch.parent.ROSLaunchParent(uuid, ["/opt/ros/noetic/share/realsense2_camera/launch/rs_t265.launch"])
    frames = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ubuntu/catkin_ws/src/climbing_robot/launch/frames.launch"])
    
    while not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message('startTracking', BoolParameter, timeout=None)
            if msg.value:
                realsense.start()
                rospy.wait_for_message('/camera/odom/sample', Odometry, timeout=None)
                frames.start()
            elif not msg.value:
                frames.shutdown()
                realsense.shutdown()
        except rospy.ROSInterruptException:
            pass