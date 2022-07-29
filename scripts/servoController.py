#!/usr/bin/env python3

import rospy
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

fsPIN=18
bsPIN=23
fwPIN=24
bwPIN=25

factory=PiGPIOFactory()

fs = Servo(fsPIN, min_pulse_width=0.0005, max_pulse_width=0.00235, frame_width=0.003003003, pin_factory=factory)
bs = Servo(bsPIN, min_pulse_width=0.0005, max_pulse_width=0.00235, frame_width=0.003003003, pin_factory=factory)
fw = Servo(fwPIN, min_pulse_width=0.001456, max_pulse_width=0.00175, frame_width=0.02, pin_factory=factory)
bw = Servo(bwPIN, min_pulse_width=0.001456, max_pulse_width=0.00175, frame_width=0.02, pin_factory=factory)

def callback(msg):
    fs.value=msg.data[0]
    bs.value=msg.data[1]
    fw.value=msg.data[2]
    bw.value=msg.data[3]

def idle():
    rospy.init_node('Controller')
    while not rospy.is_shutdown():
        rospy.Subscriber('wheelAndSteer', Float32MultiArray, callback)

if __name__ == '__main__':
    try:
        rospy.wait_for_message("/camera/odom/sample", Odometry)
        idle()
    except rospy.ROSInterruptException: 
        fs.value=0
        bs.value=0
        fw.value=0
        bw.value=0
        pass
