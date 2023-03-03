"""
Tests jetson to controllers by publishing velocity commands
"""

import rospy
from geometry_msgs.msg import Twist

motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
twist = Twist()
twist.linear.y = 0
twist.linear.z = 0
twist.angular.x = 0
twist.angular.y = 0
twist.angular.z = 1.2
twist.linear.x = 1.2

rospy.init_node('motor_controller_test', anonymous=True)

while not rospy.is_shutdown():
    motor_pub.publish(twist)