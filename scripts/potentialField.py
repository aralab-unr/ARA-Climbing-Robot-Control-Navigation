#!/usr/bin/env python2
import rospy
import os
import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped, Pose, Quaternion, Vector3
from climbing_robot.msg import Path
import tf
import numpy as np
import std_msgs.msg
from sys import exit

def potentialField(msg):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10)

    pub = rospy.Publisher('wheelAndSteer', std_msgs.msg.Float32MultiArray, queue_size=1)
    controller = std_msgs.msg.Float32MultiArray()
    controller.data = [1500,1500,1600,1600]
    pub.publish(controller)

    vr_max = 0.25# vr_max = 0.1267                         # set maximum of robot velocity 
    vr = 0
    wheel_rotation_max = np.pi / 3                     # set maximum turning angle

    i = 0
    end = False
    
    while not end:
        qv = waypoint = msg.path[i]

        broadcaster = tf2_ros.StaticTransformBroadcaster()  
        waypoint.child_frame_id = 'waypoint'
        # waypoint = TransformStamped()
        
        # waypoint.header.stamp = rospy.Time.now()
        # waypoint.header.frame_id = 'camera_odom_frame'
        

        # waypoint.transform.translation = qv.translation
        # waypoint.transform.rotation = qv.rotation
        broadcaster.sendTransform(waypoint)

        try:
            trans = tfBuffer.lookup_transform('waypoint', 'robot_pose_frame', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        dist = np.linalg.norm([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
    
        v_rd = np.sqrt(np.square(trans.transform.translation.x))
        v_rd = np.minimum(v_rd,vr_max)

        if vr < v_rd:
            vr += 0.005
            if vr > v_rd:
                vr = v_rd
        elif vr > v_rd:
            vr -= 0.005
            if vr < v_rd:
                vr = v_rd
        if yaw > 0:
            wheel_rotation = np.minimum(yaw, wheel_rotation_max)
        elif yaw < 0:
            wheel_rotation = np.maximum(yaw, -wheel_rotation_max)

        if(dist < 0.05):
            i += 1
            if qv.translation == targets[len(targets)-1].translation:
                vr = 0
                wheel_rotation = 0
                end = True

        fs = int(1500 - wheel_rotation / wheel_rotation_max * 600)
        bs = int(1500 + wheel_rotation / wheel_rotation_max * 600)
        fw = int(1600 + vr / vr_max * 800)
        bw = int(1600 - vr / vr_max * 800)

        os.system('clear')
        print("=====================================")
        print("waypoint pos:", waypoint.transform.translation.x, waypoint.transform.translation.y, waypoint.transform.translation.z)
        print("distance from waypoint (xyz):", trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
        print("distance from waypoint (norm):", dist)
        print("velocity:", vr)
        print("rotation towards waypoint (Euler): %f %f %f" %(roll, pitch, yaw))
        print("turning angle:", wheel_rotation)
        print("wheel steering input:", fs, bs)
        print("wheel motor input:", fw, bw)
        print("=====================================")

        controller.data = [fs,bs,fw,bw]
        pub.publish(controller)

        rate.sleep()

def idle():
    rospy.init_node('Controller')
    while not rospy.is_shutdown():
        rospy.Subscriber('generated_path', Path, potentialField)
        rospy.spin
if __name__ == '__main__':
    try:
        idle()
    except rospy.ROSInterruptException: pass