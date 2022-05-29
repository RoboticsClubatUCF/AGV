#!/usr/bin/env python

import rospy
from pynput import keyboard
import ugv_msg.msg as ugv

rc_pub = rospy.Publisher('/choo_2/rc', ugv.RC, queue_size=100)  # creating publisher for topic /cmd_vel (that the robot subscribes to)

rc = ugv.RC()
rc.right_x = 0
rc.left_x = 0
rc.right_y = 0
rc.left_y = 0

# tells us when the Twist message needs to be published again
change_flag = False

def on_press(key):
    ''' Every time a key is pressed, create and publish a twist message (if key pressed was an arrow key) '''

    global change_flag
    global rc

    try:
        k = key.char
    except AttributeError:
        k = key.name

    if k == 'e':
        rc.switch_e = (rc.switch_e == False)    # invert
        rospy.logdebug("E-STOP: {}".format(rc.switch_e))
    if k == 'd':
        rc.switch_d = (rc.switch_d == False)    # invert
        rospy.logdebug("AUTO: {}".format(rc.switch_d))

    publish()

def on_release(key):
    pass

def publish():

    rc_pub.publish(rc)


def main():

    # TODO: add CTRL+C signal handler. 

    rospy.init_node('rc_controller', anonymous=True, log_level=rospy.DEBUG) # node that will handle sending commands
    listener = keyboard.Listener()  # pynput keyboard listener for catching arrow key input

    # when key is pressed, run on_press(), when a key is released, run on_release()
    with keyboard.Listener(
        on_press=on_press,  
        on_release=on_release) as listener: 
        listener.join()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rc_pub.publish(rc)

        rate.sleep()

if __name__=='__main__':
    main()
