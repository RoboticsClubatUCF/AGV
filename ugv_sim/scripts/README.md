# /src
Python scripts used by this package.

## motors.py
A node that publishes Twist messages on `/cmd_vel` to control the simulated differential drive base on bowser2. Listens to the keyboard using pynput (installed through pip).

Run with:
> rosrun ugv_sim motors.py

## pose_publisher.py
Publishes PoseWithCovarianceStamped messages by modifying and publishing the nav_msgs/Odometry messages published by the differential drive Gazebo plugin on bowser2. 

Subscribed topics:
* /bowser2/odom

Published topics:
* /pose 