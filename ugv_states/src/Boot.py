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

class DataStream():

    def __init__(self, label, msg_type, topic, hz=None):

        self.label = label
        self.msg_type = msg_type
        self.topic = topic
        self.hz = hz if hz else None

        self.flag = False        

class Boot(smach.State):
    """
    BOOT ensures that we have all necessary data streams (from sensors/rc control/whatever) before passing us to STANDBY.
    Any other 'preflight' checks should be done here, as well.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['boot_success', 'None'])

        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

        # define all required data streams
        self.streams = [DataStream('RC', ugv.RC, '/choo_2/rc'),
                        DataStream('ODOM', nav.Odometry, '/zed/zed_node/odom'),
                        DataStream('LIDAR', sens.PointCloud2, '/velodyne_points'),
                        DataStream('IMU', sens.Imu, '/vectornav/IMU'),
                        DataStream('GPS', sens.NavSatFix, '/fix')]

        # create subscriber for each data stream
        for stream in self.streams:
            rospy.Subscriber(stream.topic, stream.msg_type, callback=self.stream_callback, callback_args=(stream, stream.topic))

    def execute(self, userdata):

        self.state_pub.publish("BOOT")

        # configure timer to output status of subscribers every 2 secs
        status_timer = rospy.Timer(rospy.Duration(2), self.timer_status_callback)

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            # transition to STANDBY if all sensor streams are up
            if all(stream.flag == True for stream in self.streams):
                rospy.logdebug("All sources up. Transitioning to STANDBY.")
                # end the status timer
                status_timer.shutdown()
                return 'boot_success'

            rate.sleep()

    def timer_status_callback(self, event):

        rospy.logdebug("Sensor stream status:")
        for stream in self.streams:
            rospy.logdebug("{}\t({}):\t\t{} ".format(stream.label, stream.topic, stream.flag))
        rospy.logdebug("---------------------------------------")

    def stream_callback(self, msg, args):

        # unpack args tuple
        stream = args[0]
        topic = args[1]

        if msg._type == stream.msg_type._type and topic == stream.topic:
            stream.flag = True

