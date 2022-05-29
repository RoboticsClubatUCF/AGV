#!/usr/bin/env python

''' Acts as a serial bridge to the RPi Pico
'''
# ros imports
import rospy
from rospy_message_converter import json_message_converter

# python imports
import serial
import math
from typing import Tuple

# message imports
import std_msgs.msg as std
import geometry_msgs.msg as geom
import nmea_msgs.msg as nmea
import ugv_msg.msg as ugv

# Wheel radius (meters)
WHEEL_RADIUS = 0.10795
METERS_PER_REV = WHEEL_RADIUS * math.pi * 2
REVS_PER_METER = 1 / METERS_PER_REV

def ticks_to_message(input):
    # type: (list[str]) -> Tuple[std.Int64, std.int64]
    """
    Converts strings in the encoder format to two encoder (Int64) messages.
    Format: format: $ENC <encoder1> <encoder2>
    """

    l_ticks = std.Int64()
    r_ticks = std.Int64()

    # ticks are reported strangely
    try:
        l_ticks.data = int(input[1])    # encoder 2 on PICO
        r_ticks.data = int(input[0])    # encoder 1 on PICO
    except ValueError as e:
        rospy.log_err('Invalid ticks data')
        return None

    return (l_ticks, r_ticks)

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

def str_to_RC_message(rc_list):
    """
    Converts strings in RC format to RC messages.
    format: $RCX <RJ_X> <RJ_Y> <LJ_Y> <LJ_X> <ESTOP> <DIAL> <AUTO>
    """

    rc_msg = ugv.RC()

    # populate header
    rc_msg.header = std.Header()
    rc_msg.header.stamp = rospy.Time.now()
    rc_msg.header.frame_id = "" # frame is meaningless in this context
    # populate the sticks
    right_x = int(rc_list[0])
    left_x = int(rc_list[3])

    if right_x < 1000:
        right_X = 1000
    elif right_x > 2000:
        right_X = 2000
    if left_x < 1000:
        left_X = 1000
    elif left_x > 2000:
        left_X = 2000

    # populate the switches
    rc_msg.switch_e = (int(rc_list[4]) <= 1500) # E-STOP
    rc_msg.switch_g = int(rc_list[5])
    rc_msg.switch_d = (int(rc_list[6]) >= 1500) # AUTO

    return rc_msg

prev_estop = False
# TODO: send a message to activate lights
def rc_callback(message, serial):

    global prev_estop

    # E-STOP thrown when it wasn't before
    if message.switch_e and not prev_estop:
        estp = "$STP\n"
        encoded = estp.encode('utf-8')
        serial.write(encoded)
    # E-STOP not thrown when it was before
    if not message.switch_e and prev_estop:
        estp = "$GO\n"
        encoded = estp.encode('utf-8')
        serial.write(encoded)
    
    prev_estop = message.switch_e

def main():

    rospy.init_node('arduino_bridge', anonymous=True, log_level=rospy.DEBUG)

    # publishers for data streams FROM the board
    l_tick_pub = rospy.Publisher('/choo_2/left_ticks', std.Int64, queue_size=1)
    r_tick_pub = rospy.Publisher('/choo_2/right_ticks', std.Int64, queue_size=1)
    err_pub = rospy.Publisher('/choo_2/arduino_bridge/errors', std.String, queue_size=1)
    rc_pub = rospy.Publisher('/choo_2/rc', ugv.RC, queue_size=1)

    # attempt to establish serial connection with the board
    _port = '/dev/arduino'
    ser = serial.Serial(
        port=_port,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    rospy.logdebug("Serial Connection established on {}".format(_port))

    # subscribers
    # TODO: pass each of these the serial queue, not the actual serial connection
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', geom.Twist, callback=cmd_vel_cb, callback_args=(ser))
    rc_sub = rospy.Subscriber("/choo_2/rc", ugv.RC, callback=rc_callback,callback_args=(ser))

    if not ser.is_open:
        rospy.logerr("Couldn't open serial port.")
        return

    while not rospy.is_shutdown():

        # read input lines
        try:
            input = ser.readline()
        except serial.SerialException as e:
            rospy.logerr_once("Issue with serial read: {}".format(e))
            continue
        # attempt to decode input, strip whitespace
        print(input)
        try:
            input = input.decode().strip()
        except UnicodeDecodeError as e:
            rospy.logerr("Decoding error: {}".format(e))
            continue

        # 'tokenize' by spaces
        input_split = input.split(" ")

        prefix = input_split[0][0:4]    # pull prefix from messages to switch

        # RC receiver data
        if prefix == '$RCX':
            rc_msg = str_to_RC_message(input_split[1:])
            rc_pub.publish(rc_msg)
        # serial data from either motor controller
        elif prefix == '$MC1' or prefix == '$MC2':
            pass
        # encoder ticks to be published
        elif prefix == '$ENC':
            # get ticks messages from string
            left_tick, right_tick = ticks_to_message(input_split[1:])
            # publish tick messages
            if left_tick is not None:
                l_tick_pub.publish(left_tick)
            if right_tick is not None:
                r_tick_pub.publish(right_tick)
        # publish error messages
        elif prefix == '$ERR':
            msg = std.String()
            msg.data = "".join(input_split[1])
            err_pub.publish(msg)
        else:
            rospy.logdebug("Unhandled prefix: {}".format(input))

        # TODO: switch to sending queue, rather than callback senders


if __name__=='__main__':
    main()
