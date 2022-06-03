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

class Estop(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['end', 'standby'],
                                   output_keys=[])

        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

        self.ESTOP = True

    def execute(self, userdata):

        self.state_pub.publish("ESTOP")
        self.ESTOP = True

        # wait until execute to initialize subscribers so that multiple states can listen to same topic names without clashing
        rc_sub = rospy.Subscriber("/choo_2/rc", ugv.RC, callback=self.rc_callback)

        while not rospy.is_shutdown():

            if self.ESTOP == False:
                rospy.logdebug("ESTOP cleared, returning to STANDBY.")
                return 'standby'
            
    def rc_callback(self, msg):

        if msg.switch_e is not None:
            self.ESTOP = msg.switch_e
