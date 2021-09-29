#!/bin/bash

# install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full

# so that I can source my workspace when I want
if ! grep -q "_ros1()" /home/$USER/.bashrc; then
	echo 
"
_ros1() {

	source /opt/ros/melodic/setup.bash
	source /home/$USER/catkin_ws/devel/setup.bash
	echo \"workspace: ros1 $ROS_DISTRO\"

	# Gazebo environment variables
	export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/$USER/catkin_ws/src/simple_sim/worlds
	export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/$USER/catkin_ws/src/simple_sim/models
}" >> /home/$USER/.bashrc
fi
 
source /opt/ros/melodic/setup.bash

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
sudo apt-get install ros-melodic-catkin python-catkin-tools -y

sudo rosdep init
rosdep update

sudo apt install ros-melodic-navigation -y
sudo apt install ros-melodic-slam-toolbox -y

if ! [ -d "/home/$USER/catkin_ws" ]
then 
	echo "creating /home/$USER/catkin_ws"
	mkdir -p /home/$USER/catkin_ws/src
fi

#cd /home/$USER/catkin_ws
#catkin_make

source /home/$USER/catkin_ws/devel/setup.bash
source /opt/ros/melodic/setup.bash

CATKIN=/home/$USER/catkin_ws
CATKIN_SRC=/home/$USER/catkin_ws/src

# number of cores (so we don't murder low power computers during the build)
declare -i total_cores
declare -i build_cores
total_cores=$(nproc --all)
build_cores=$(expr $total_cores / 2 ) 

# clone my working repos
cd $CATKIN_SRC
git clone https://github.com/wesfletch/simple_sim.git -b melodic

# elevation_mapping and it's dependencies
sudo apt install ros-$ROS_DISTRO-grid-map

cd $CATKIN_SRC
# if we don't have kindr in catkin_ws, pull it from github
if ! [ -d "$CATKIN_SRC/kindr" ]
then
	git clone https://github.com/ANYbotics/kindr.git
	cd ..
	catkin build kindr -j $build_cores
fi

cd $CATKIN_SRC
if ! [ -d "$CATKIN_SRC/kindr_ros" ]
then
	git clone https://github.com/ANYbotics/kindr_ros.git
	cd ..
	catkin build kindr_ros -j $build_cores
fi

cd $CATKIN_SRC
if ! [ -d "$CATKIN_SRC/perception_pcl" ]
then
	git clone https://github.com/ros-perception/perception_pcl --branch 1.7.2
	# pulling the version of perception_pcl marked for ROS Noetic (latest ROS1 release)
	cd ..
	catkin build perception_pcl -j $build_cores
fi

cd $CATKIN_SRC
if ! [ -d "$CATKIN_SRC/elevation_mapping" ]
then
	git clone https://github.com/anybotics/elevation_mapping.git --branch release
	# have to pull down an old version of elevation_mapping since PCL 1.11.1 isn't available for Ubuntu 18.04	(https://github.com/ANYbotics/elevation_mapping/issues/151)
	cd ..
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
	catkin build -j $build_cores
fi


