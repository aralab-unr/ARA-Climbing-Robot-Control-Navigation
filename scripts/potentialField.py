#!/usr/bin/env python3
import rospy
import os
import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped, Pose, Quaternion, Vector3
from dynamic_reconfigure.msg import BoolParameter
from climbing_robot.msg import Path
import tf
import numpy as np
import std_msgs.msg

def potentialField(msg):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10)

    # Publisher to topic arduino is listening to
    # controller.data format:[front wheel steering, back wheel steering, front wheel rotation, back wheel rotation]
    pub = rospy.Publisher('wheelAndSteer', std_msgs.msg.Float32MultiArray, queue_size=1)
    reachPub = rospy.Publisher('Reach',BoolParameter,queue_size=10)
    controller = std_msgs.msg.Float32MultiArray()
    controller.data = [0,0,0,0]         # set all to 0
    pub.publish(controller)
    print("start")
    

    vr_max = 0.25                               # set maximum of robot velocity (m/s)
    vr = 0
    wheel_rotation_max = np.pi / 2              # set maximum wheel rotation angle (rad)
    reached = 0.1                              # set radius to reach waypoint

    i = 0                                       # initialize waypoint iterator
    end = BoolParameter()
    end.value = False
    reachPub.publish(end)
    
    while not end.value:
        connecting = True
        trans = TransformStamped()
        while connecting:       # get transformation between robot frame and waypoint frame (1 : len(msg.path))
            try:
                # trans = tfBuffer.lookup_transform('robot_pose_frame', str(i+1), rospy.Time())
                trans = tfBuffer.lookup_transform('camera_pose_frame', str(i+1), rospy.Time())
                connecting = False
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("connecting")
                continue

        # Find normal distance between robot and waypoint
        dist = np.linalg.norm([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        # Convert quaternion to euler
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])

        # set velocity controller of robot
        v_rd = dist
        # limit max velocity
        v_rd = np.minimum(v_rd,vr_max)
        vr = v_rd

        # limit acceleration of robot
        # if vr < v_rd:
        #     vr += 0.005
        #     if vr > v_rd:
        #         vr = v_rd
        # elif vr > v_rd:
        #     vr -= 0.005
        #     if vr < v_rd:
        #         vr = v_rd

        # limit maximum rotation (correcting for +/- angles)
        if yaw > 0:
            wheel_rotation = np.minimum(yaw, wheel_rotation_max)
        elif yaw < 0:
            wheel_rotation = np.maximum(yaw, -wheel_rotation_max)

        # Iterate to next waypoint if within reach radius of waypoint
        if(dist < reached):
            i += 1
            # wheel_rotation = 0
            # If reached final waypoint, set controls to 0 and end loop
            if msg.path[i].transform.translation == msg.path[len(msg.path)-1].transform.translation:
                vr = 0
                wheel_rotation = 0
                end = BoolParameter()
                end.value = True
                reachPub.publish(end)

        # convert control values to servo value range 
            # Steering 1500 = 0, 900 - 2100
            # Wheel rotation 1600 = 0, 800 - 2400
        fs = int(wheel_rotation / wheel_rotation_max)
        bs = int(wheel_rotation / wheel_rotation_max)
        fw = int(vr / vr_max)
        bw = int(vr / vr_max)

        controller.data = [fs,bs,fw,bw]
        pub.publish(controller)

        rate.sleep()

def idle():
    rospy.init_node('Potential Field')
    while not rospy.is_shutdown():
        path=rospy.wait_for_message('generated_path', Path)
        potentialField(path)
        
if __name__ == '__main__':
    try:
        idle()
    except rospy.ROSInterruptException: 
        pass