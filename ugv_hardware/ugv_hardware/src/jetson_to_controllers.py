#!/usr/bin/env python

""" Allows us to send commands directly to the motor controllers """

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
MAX_MOTOR_RPM = 375.0
MAX_RPM_VALUE = 1000.0

def write_string(input, serials):
    """
    Writes a given string to the motor controllers.
    """
    for serial in serials:
        encoded = input.encode('utf-8')
        serial.write(encoded)

AUTO_SWITCH = False
prev_estop = False
def rc_callback(message, args):

    global AUTO_SWITCH
    global prev_estop

    ser1 = args[0]
    ser2 = args[1]

    AUTO_SWITCH = message.switch_d

    # if the e-stop is cleared
    if not message.switch_e and prev_estop:
        rospy.logdebug("Clearing E-STOP!")
        write_string("!MG\n\r", (ser1, ser2))
    # Auto-nav mode; do nothing with these messages
    elif message.switch_d:
        return

    prev_estop = message.switch_e

    # handle joystick inputs
    left_rpm = int((message.left_x - 1500)*(MAX_MOTOR_RPM / MAX_RPM_VALUE))
    right_rpm = int((message.right_x - 1500)*(MAX_MOTOR_RPM / MAX_RPM_VALUE))
    string = "!M " + str(left_rpm) + " " + str(right_rpm) + "\r"

    write_string(string, (ser1, ser2))

def cmd_vel_cb(cmd_vel, args):

    global AUTO_SWITCH

    # Auto-nav mode is off
    if not AUTO_SWITCH:
        AUTO_SWITCH = False
        return

    left_velocity =  cmd_vel.linear.x - (0.5 * cmd_vel.angular.z * WHEEL_BASE)
    right_velocity = cmd_vel.linear.x + (0.5 * cmd_vel.angular.z * WHEEL_BASE)
    rospy.logdebug("3 LEFT_VEL {} RIGHT_VEL {}".format(left_velocity, right_velocity))

    # convert m/s to RPM
    left_rpm = left_velocity * REVS_PER_METER * 60
    right_rpm = right_velocity * REVS_PER_METER * 60
    rospy.logdebug("4 LEFT_RPM {} RIGHT_RPM {}".format(left_rpm, right_rpm))

    # Serial Write
    string = "!M " + str(left_rpm) + " " + str(right_rpm) + "\r"
    # write_string(string, (ser1, ser2))
    encoded = string.encode('utf-8')
    args[0].write(encoded)
    args[1].write(encoded)

def main():     

    rospy.init_node('motor_controller_bridge', anonymous=True, log_level=rospy.DEBUG)
    _port = '/dev/ttyACM0'
    ser1 = serial.Serial(
        port=_port,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    rospy.logdebug("Serial Connection established on {}".format(_port))

    _port = '/dev/ttyACM1'
    ser2 = serial.Serial(
        port=_port,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    rospy.logdebug("Serial Connection established on {}".format(_port))

    # Subscribers
    # rc_sub = rospy.Subscriber("/choo_2/rc", ugv.RC, callback=rc_callback,callback_args=(ser1,ser2))
    # cmd_vel_sub = rospy.Subscriber('/cmd_vel', geom.Twist, callback=cmd_vel_cb, callback_args=(ser1,ser2))
    
    # Subscribers
    rc_sub = rospy.Subscriber("/choo_2/rc", ugv.RC, callback=rc_callback,callback_args=(ser1,ser2))
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', geom.Twist, callback=cmd_vel_cb, callback_args=(ser1,ser2))

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        # read input lines
        try:
            input = ser1.readline()
        except serial.SerialException as e:
            rospy.logerr_once("Issue with serial read: {}".format(e))
            continue
        # attempt to decode input, strip whitespace
        rospy.logdebug("MC1 says: {}".format(input))

        # read input lines
        try:
            input = ser1.readline()
        except serial.SerialException as e:
            rospy.logerr_once("Issue with serial read: {}".format(e))
            continue
        # attempt to decode input, strip whitespace
        rospy.logdebug("MC2 says: {}".format(input))

        rate.sleep()

if __name__=='__main__':
    main()