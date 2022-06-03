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

class Manual(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['rc_un_preempt', 'error'])

        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

        # subscribe to RC commands
        # TODO: make/find an RC channels message
        rospy.Subscriber('/choo_2/rc', std.String, callback=self.rc_callback)

        # publish motor commands for the base_controller to actuate
        self.motor_pub = rospy.Publisher('/cmd_vel', geom.Twist, queue_size=10)
    
    def execute(self, userdata):

        self.state_pub.publish("MANUAL")

        while not rospy.is_shutdown():
            pass

    def rc_callback(self, data):

        # convert raw RC to twist (if that's the approach we're taking)

        # publish on cmd_vel with self.motor_pub
        pass