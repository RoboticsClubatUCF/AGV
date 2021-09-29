#!/usr/bin/env python3

''' very simple script: listens to the nav_msgs/Odometry published by sim model
    and publishes the 'pose' section of the message as a PoseWithCovarianceStamped' '''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

pose_pub = rospy.Publisher("/pose", PoseWithCovarianceStamped, queue_size=100)

# pose is the message we're creating
pose = PoseWithCovarianceStamped()

# callback: listens on <ODOM_TOPIC> and runs every time it receives a message
# updates the pose variable which we publish later
def odom_callback(odom):

    # create header
    pose.header = odom.header
    pose.header.stamp = rospy.Time.now()

    # just copy the odometry message pose into our PoseWithCovarianceStamped
    pose.pose = odom.pose
    
    # zero out covariance matrix, since our sensor is "perfect"
    pose.pose.covariance = [0, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 0]

if __name__ == '__main__':
    
    rospy.init_node('pose_publisher', anonymous=True)

    # listens to odometry published by sim
    rospy.Subscriber("/bowser2/odom", Odometry, odom_callback)

    rate = rospy.Rate(10) # Hz

    while not rospy.is_shutdown():

        # publish the message
        pose_pub.publish(pose)

        rate.sleep()

    # rospy.spin()
