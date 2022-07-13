#!/usr/bin/env python3
import PySimpleGUI as sg
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from dynamic_reconfigure.msg import BoolParameter
from tf.transformations import euler_from_quaternion
import roslaunch

currentPosition = Point()
currentOrientation = Quaternion()  
reached = True

def toEuler(quat):
    quatList = [quat.x, quat.y, quat.z, quat.w]
    return euler_from_quaternion(quatList)

def odomCallback(msg):
    global currentPosition
    global currentOrientation
    currentPosition = msg.pose.pose.position
    currentOrientation = msg.pose.pose.orientation

def reachCallback(msg):
    global reached
    reached = msg.value

def GUI():
    # Create an event loop
    rospy.init_node('GUI')
    default_font = ('normal', 15)
    rate=rospy.Rate(10)    
    pub=rospy.Publisher('Target', Point,queue_size=10)
    startTracking=rospy.Publisher('startTracking', BoolParameter,queue_size=10)
    track = BoolParameter()
    track.value=True
    startTracking.publish(track)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/aralab/catkin_ws/src/climbing_robot/launch/baseLaunch.launch"])
    launch.start()
    rospy.loginfo("started")
   
    window = sg.Window("Climbing Robot GUI", layout=[   [sg.Text('Robot Position:',justification='center'),
                                                            sg.Text(('X: '),key='XPOS',justification='center',pad=2), 
                                                            sg.Text(('Y: '),key='YPOS',justification='center',pad=2), 
                                                            sg.Text(('Z: '),key='ZPOS',justification='center',pad=2)],
                
                                                        [sg.Text('Robot Position:',font=default_font,justification='center'),
                                                            sg.Text(('Roll: '),key='ROLL',justification='center',pad=2), 
                                                            sg.Text(('Pitch: '),key='PITCH',justification='center',pad=2), 
                                                            sg.Text(('Yaw:  '),key='YAW',justification='center',pad=2)],

                                                        [sg.Button("Post Target", key='POST'), 
                                                            sg.Text('X:',justification='center',pad=((16,3),0)), 
                                                            sg.Input(s=5,key='XIN',justification='center',pad=((2,4),0)),
                                                            sg.Text('Y:',justification='center',pad=((2,3),0)), 
                                                            sg.Input(s=5,key='YIN',justification='center',pad=((2,5),0)),
                                                            sg.Text('Z:',justification='center',pad=((2,3),0)),
                                                            sg.Input(s=5,key='ZIN',justification='center',pad=((2,3),0))],

                                                        [sg.Button("Exit")]], font=default_font)
    while not rospy.is_shutdown():
    # while True:
        event, values = window.read(timeout=0)
        rospy.Subscriber('/camera/odom/sample',Odometry,odomCallback)      
        rospy.Subscriber('Reach',BoolParameter,reachCallback)  

        (roll, pitch, yaw) = toEuler(currentOrientation)
        window['XPOS'].update('X: %.2f' %currentPosition.x) 
        window['YPOS'].update('Y: %.2f' %currentPosition.y) 
        window['ZPOS'].update('Z: %.2f' %currentPosition.z)
        window['ROLL'].update('Roll: %.2f' %roll)
        window['PITCH'].update('Pitch: %.2f' %pitch) 
        window['YAW'].update('Yaw: %.2f' %yaw)
                            
        if event == "POST":
            try:
                x=float(window['XIN'].get())
                y=float(window['YIN'].get())
                z=float(window['ZIN'].get())
                target=Point(x,y,z)
                pub.publish(target)
                window['POST'].update(disabled=True)
                global reached
                reached = False
            except ValueError:
                sg.popup("Please input a numerical values.", no_titlebar=True, font=default_font)

        if event == "Exit" or event == sg.WIN_CLOSED:
            break
        if reached:
            window['POST'].update(disabled=False)
        rate.sleep()
    window.close()
    launch.shutdown()

if __name__ == '__main__':
    try:
        GUI()
    except rospy.ROSInterruptException:
        pass
    