#!/usr/bin/env python3

''' Acts as a serial bridge to the RPi Pico
'''
# ros imports
import rospy
from rospy_message_converter import json_message_converter

# python imports
import serial
from typing import Tuple

# message imports
import std_msgs.msg as std
import geometry_msgs.msg as geom
import nmea_msgs.msg as nmea
import ugv_msg.msg as ugv

def str_to_command(input: list) -> rov.Cmd:

    """
    Input format: X Y Z start cancel shutdown rc_preempt pose_preempt
    """

    cmd = rov.Cmd()

    cmd.target = geom.Point()
    try:
        cmd.target.x = float(input[0])
        cmd.target.y = float(input[1])
        cmd.target.z = float(input[2])
    except ValueError as e:
        rospy.logerr(e)
        return None

    # flags (TODO: need to be cast?)
    cmd.start.data          = (input[3] == 'True')
    cmd.cancel.data         = (input[4] == 'True')
    cmd.shutdown.data       = (input[5] == 'True')
    cmd.rc_preempt.data     = (input[6] == 'True')
    cmd.pose_preempt.data   = (input[7] == 'True')

    return cmd

def str_to_rc_command(input: list) -> rov.Cmd:

    """
    Converts "rc burst" style strings into Cmd messages
    """

    cmd = rov.Cmd()

    # rc fields
    cmd.rc.forward  = int(input[0])
    cmd.rc.reverse  = int(input[1])
    cmd.rc.left     = int(input[2])
    cmd.rc.right    = int(input[3])

    # "burst" messages always throw this flag to set state machine to MANUAL
    cmd.rc_preempt.data = True
    # all other flags are false
    cmd.start.data          = False
    cmd.cancel.data         = False
    cmd.shutdown.data       = False
    cmd.pose_preempt.data   = False
    
    return cmd    

def str_to_rc_message(input):

    rc = ugv.RC()

def ticks_to_message(input: list) -> Tuple[std.Int64, std.Int64]:

    l_ticks = std.Int64()
    r_ticks = std.Int64()

    # ticks are reported strangely
    try:
        l_ticks.data = int(input[3])    # encoder 2 on PICO
        r_ticks.data = int(input[1])    # encoder 1 on PICO
    except ValueError as e:
        rospy.log_err('Invalid ticks data')
        return None

    return (l_ticks, r_ticks)

def tlm_cb(tlm: rov.Telemetry, ser: serial.Serial) -> None:

    ''' when we receive telemetry messages, pass them through to the pico to be transmitted '''

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
    rc_msg.right_x = int(rc_list[0])
    rc_msg.right_y = int(rc_list[1])
    rc_msg.left_y = int(rc_list[2])
    rc_msg.left_x = int(rc_list[3])
    # populate the switches
    rc_msg.switch_e = (int(rc_list[4]) >= 1500)
    rc_msg.switch_g = int(rc_list[5])
    rc_msg.switch_d = (int(rc_list[6]) >= 1500)

    return rc_msg

def main():

    rospy.init_node('arduino_bridge', anonymous=True, log_level=rospy.DEBUG)

    # publishers for data streams FROM the pico
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
    tlm_sub = rospy.Subscriber('/telemetry', rov.Telemetry, callback=tlm_cb, callback_args=(ser))
    pwm_sub = rospy.Subscriber('/motors', rov.Motors, callback=pwm_cb, callback_args=(ser))
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', geom.Twist, callback=cmd_vel_cb, callback_args=(ser))

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
        elif prefix == '$MC1' or prefix == '%MC2':
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
