#!/usr/bin/env python

""" Allows us to send commands directly to the esp-32 """

# GPIO imports
import serial

# ros imports
import copy
import rospy

# python imports
import serial
import math
from typing import Tuple

# message imports
import std_msgs.msg as std
import geometry_msgs.msg as geom
import ugv_msg.msg as ugv

WHEEL_BASE = 0.67
WHEEL_RADIUS = 0.10795
METERS_PER_REV = WHEEL_RADIUS * math.pi * 2
REVS_PER_METER = 1 / METERS_PER_REV
#originally 1000
MAX_MOTOR_RPM = 375
MAX_RPM_VALUE = 375

global ser
ser = serial.Serial(
        port='/dev/ttyTHS2',
        baudrate=115200,
    )


def write_string(input, serial):
    """
    Writes a given string to the serial port.
    """
    #encoded = input.encode('ascii')
    serial.write(input)


AUTO_SWITCH = False
prev_estop = False
# def rc_callback(message, args):

#     global AUTO_SWITCH
#     global prev_estop

#     new_ser = args[0]

#     AUTO_SWITCH = message.switch_d

#     # if the e-stop is cleared
#     if not message.switch_e and prev_estop:
#         rospy.logdebug("Clearing E-STOP!")
#         write_string("!MG\n\r", (new_ser))
#     # Auto-nav mode; do nothing with these messages
#     elif message.switch_d:
#         return

#     prev_estop = message.switch_e

#     # handle joystick inputs
#     left_rpm = int((message.left_x - 1500)*(MAX_MOTOR_RPM / MAX_RPM_VALUE))
#     right_rpm = int((message.right_x - 1500)*(MAX_MOTOR_RPM / MAX_RPM_VALUE))
#     rospy.logdebug("joystick : left : " + str(left_rpm) + " right : " + str(right_rpm))
#     # write command to motor controllers


def cmd_vel_cb(cmd_vel, args):

    global AUTO_SWITCH

    # # Auto-nav mode is off
    # if AUTO_SWITCH == False:
    #     return

    left_velocity  =  cmd_vel.linear.x - (0.5 * cmd_vel.angular.z * WHEEL_BASE)
    right_velocity =  cmd_vel.linear.x + (0.5 * cmd_vel.angular.z * WHEEL_BASE)

    # convert m/s to RPM
    left_rpm = int(left_velocity * REVS_PER_METER * 60)
    right_rpm = int(right_velocity * REVS_PER_METER * 60)
    rospy.logdebug("ALIBDLJFNLS:AODNFOJKSDNF LEFT: {} RIGHT: {}".format(left_rpm, right_rpm))
    rospy.logdebug("auto : left : " + str(left_rpm) + " right : " + str(right_rpm))
    # Allocate 10 bits for left velocity, 10 bits for the right, 20 bits in total
    cmd_str = str(left_rpm) + '|' + str(right_rpm) + "\n"
    write_string(cmd_str, ser)


def run():     
    rospy.init_node('motor_controller_bridge', anonymous=True, log_level=rospy.DEBUG)
    
    # Subscribers
    # rc_sub = rospy.Subscriber("/choo_2/rc", ugv.RC, callback=rc_callback, callback_args=(ser))
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', geom.Twist, callback=cmd_vel_cb, callback_args=(ser))
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__=='__main__':
    run()