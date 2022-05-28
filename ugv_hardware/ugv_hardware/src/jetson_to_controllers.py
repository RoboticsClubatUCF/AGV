#!/usr/bin/env python

""" Allows us to send commands directly to the motor controllers"""

# ros imports
import rospy

# python imports
import serial
import math
from typing import Tuple

# message imports
import std_msgs.msg as std
import geometry_msgs.msg as geom
import ugv_msg.msg as ugv

def write_string(input, serials):
    """
    Writes a given string to the motor controllers.
    """
    for serial in serials:
        encoded = input.encode('utf-8')
        serial.write(encoded)

prev_estop = False
def rc_callback(message, serialargs):

    # if the e-stop is cleared
    if not message.switch_e and prev_estop:
        rospy.logdebug("Clearing E-STOP!")
        write_string("!MG\n\r", serialargs)
    # Auto-nav mode; do nothing with these messages
    elif message.switch_d:
        return

    prev_estop = message.switch_e
    # # handle joystick inputs
    # else:
    #     left_rpm = int((message.right_x  - 1500)*(0.4))
    #     right_rpm = int((message.left_x  - 1500)*(0.4))
    #     string = "!M " + str(right_rpm) + " " + str(left_rpm) + "\n\r"
    #     write_string(string, serialargs)
    
def cmd_vel_cb(cmd_vel):

    rpm_1 = 0
    rpm_2 = 0

    # convert m/s to RPM
    x_rpm = cmd_vel.linear.x * REVS_PER_METER * 60
    if cmd_vel.angular.z == 0.0:
        rpm_1 = x_rpm
        rpm_2 = x_rpm

    msg_dict = []

    pass

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
    rc_sub = rospy.Subscriber("/choo_2/rc", ugv.RC, callback=rc_callback,callback_args=(ser1,ser2))
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', geom.Twist, callback=cmd_vel_cb, callback_args=(ser1,ser2))
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        rate.sleep()

if __name__=='__main__':
    main()