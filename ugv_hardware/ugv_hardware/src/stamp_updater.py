#!/usr/bin/env python

"""Listens to a GPS topic, and updates the timestamp. Intended to fix the weird timestamp on the Spatial."""

# general imports
import sys
from xml.dom.minidom import Attr

# ROS system imports
import rospy

# ROS messages
import std_msgs.msg as std
import nav_msgs.msg as nav
import sensor_msgs.msg as sens
import geometry_msgs.msg as geom
import move_base_msgs.msg as move_base
import ugv_msg.msg as ugv

def callback(msg, publisher):

    try:
        msg.header.stamp = rospy.Time.now()
        publisher.publish(msg)
    except AttributeError:
        rospy.logerr("This message doesn't have a stamp.")
        return

def main():

    rospy.init_node("stamp_updater", anonymous=True, log_level=rospy.DEBUG)

    try:
        assert(len(sys.argv) > 1)
    except AssertionError:
        rospy.logerr("Wrong number of args given to timestamp_updater.")
        return

    types = {'sensor_msgs/Imu': sens.Imu,
             'sensor_msgs/NavSatFix': sens.NavSatFix,
             'nav_msgs/Odometry': nav.Odometry}

    msg_type = sys.argv[1]
    topic = sys.argv[2]
    pub_topic = topic + "/updated"

    rospy.logdebug("Subscribing to message of type {} from topic {}".format(msg_type, topic))
    rospy.logdebug("Publishing on topic {}".format(pub_topic))

    pub = rospy.Publisher(pub_topic, types[msg_type], queue_size=1)
    rospy.Subscriber(topic, types[msg_type], callback=callback, callback_args=(pub))

    rospy.spin()

if __name__=="__main__":
    main()