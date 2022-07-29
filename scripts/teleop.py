#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from pynput import keyboard

controller = Float32MultiArray()

def on_press(key):
    global controller
    try:
        if key.char == "w":
            controller.data[2]=1
            controller.data[3]=-1
        elif key.char == "s":
            controller.data[2]=-1
            controller.data[3]=1
        elif key.char == "a":
            controller.data[0]=0.5
            controller.data[1]=-0.5
        elif key.char == "d":
            controller.data[0]=-0.5
            controller.data[1]=0.5
    except AttributeError:
        print(key)

def on_release(key):
    global controller
    try:
        if key.char == "w":
            controller.data[2]=0
            controller.data[3]=0
        elif key.char == "s":
            controller.data[2]=0
            controller.data[3]=0
        elif key.char == "a":
            controller.data[0]=0
            controller.data[1]=0
        elif key.char == "d":
            controller.data[0]=0
            controller.data[1]=0
    except AttributeError:
        print(key)
 
if __name__ == '__main__':
    pub = rospy.Publisher('wheelAndSteer', Float32MultiArray, queue_size=1)
    rospy.init_node('Controller')
    r = rospy.Rate(10)

    controller.data = [0,0,0,0]
   
    try:
        listener = keyboard.Listener(
            on_press=on_press,
            on_release=on_release)
        listener.start()
        while not rospy.is_shutdown():
            pub.publish(controller)
            r.sleep()

    except rospy.ROSInterruptException:
        pass