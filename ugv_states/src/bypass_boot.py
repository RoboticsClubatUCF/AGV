#!/usr/bin/env python

# ROS system imports
import queue
import rospy
import actionlib
import smach, smach_ros

# ROS messages
import std_msgs.msg as std
import nav_msgs.msg as nav
import sensor_msgs.msg as sens
import geometry_msgs.msg as geom
import move_base_msgs.msg as move_base
import ugv_msg.msg as ugv

def main():

    rospy.init_node('bypass_boot', anonymous=True, log_level=rospy.DEBUG)

    rc_pub = rospy.Publisher('/choo_2/rc', ugv.RC, queue_size=1)
    odom_pub = rospy.Publisher('/zed/zed_node/odom', nav.Odometry, queue_size=1)
    vel_pub = rospy.Publisher('/velodyne_points', sens.PointCloud2, queue_size=1)
    fix_pub = rospy.Publisher('/choo_2/fix', sens.NavSatFix, queue_size=1)

    while not rospy.is_shutdown():
        rc = ugv.RC()
        rc_pub.publish(rc)

        odom = nav.Odometry()
        odom_pub.publish(odom)

        vel = sens.PointCloud2()
        vel_pub.publish(vel)

        fix = sens.NavSatFix()
        fix_pub.publish(fix)

if __name__=='__main__':
    main()