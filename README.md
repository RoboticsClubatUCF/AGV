# AGV 
RCCF's entries for the annual IGVC competition.

This repository contains a collection of ROS packages used in the operation of an unmanned ground vehicle.

# Setup 
These packages were intended to run on ROS Noetic, which requires Ubuntu 20.04.

1. Clone this repo into ~/catkin_ws/src with 
	' git clone https://github.com/RoboticsClubatUCF/AGV.git '

2. Run the setup script with 
	' ./setup.sh '



# Directories

**ugv_sim**
- Contains the launch files, simulation worlds, and configuration files used for RVIZ. This may also contain any scripts we use to make working with the sim easier, and submodules for simulating sensors. 

**ugv_nav**
- Contains the configuration and launch files for our navigation stack. Move-base and robot-localization are configured and launched here.

**ugv_vision**
- Contains any code we use for computer vision.

**ugv_states**
- Contains the launch files and scripts for our state machine.

**ugv_description**
- Contains any files used describe the dimensions of our robot, and any files used to describe our simulated robot.

**ugv_bringup**
- This is the "on" button. It contains launch files that turn on or "bring up" our sensors and other packages required for operation.

**ugv_msg** 
- Contains descriptions for custom ros messages.

**ugv_hardware**
- Contains any code used for sending and recieving serial data over the jetson's UART pins. Also contains any arduino code we use.



### Eat trash, loser(s).
