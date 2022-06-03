#!/usr/bin/env python

# ROS system imports
import rospy
import actionlib
import smach, smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

# ROS messages
import std_msgs.msg as std
import nav_msgs.msg as nav
import sensor_msgs.msg as sens
import geometry_msgs.msg as geom
import move_base_msgs.msg as move_base
import ugv_msg.msg as ugv

class Standby(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['rc_preempt', 'ESTOP', 'AUTO'],
                                   output_keys=[])

        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

        self.AUTO = False
        self.ESTOP = False

    def execute(self, userdata):

        self.state_pub.publish("STANDBY")

        # wait until execute to initialize subscribers so that multiple states can listen to same topic names without clashing
        rc_sub = rospy.Subscriber("/choo_2/rc", ugv.RC, callback=self.rc_callback)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            if self.ESTOP == True:
                self.ESTOP = False
                return 'ESTOP'
            elif self.AUTO == True:
                self.AUTO = False
                return 'AUTO'

            rate.sleep()

    def rc_callback(self, msg):

        if msg.switch_e is not None:
            self.ESTOP = msg.switch_e
        if msg.switch_d is not None:
            self.AUTO = msg.switch_d
