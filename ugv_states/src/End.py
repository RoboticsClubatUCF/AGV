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

class End(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'err', 'end'],
                                   input_keys=['reason'])
        
        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

    def execute(self, userdata):

        self.state_pub.publish("END")
        # kill the ROS node
        # http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown

        return 'end'

        # rospy.signal_shutdown(userdata.reason)

        # if userdata.end_status == 'success':
        #     return 'end_success'
        # elif userdata.end_status == 'err':
        #     return 'end_err'