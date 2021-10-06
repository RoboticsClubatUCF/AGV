# if ROS isn't installed, install it
if ! [ -d "/opt/ros/noetic" ]
then
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	
	sudo apt install curl # if you haven't already installed curl
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	
	sudo apt update
	sudo apt install ros-noetic-desktop-full # this takes a while
fi

source /opt/ros/noetic/setup.bash

# some ROS-related tools we may need
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# initialize rosdep
sudo rosdep init
rosdep update

# some miscellaneous tools
sudo apt install -y git
sudo apt install -y python-3 pip
pip3 install pynput

# ugv_sim dependencies:
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-slam-toolbox

# Adding environment variables to .bashrc means we don't need to source them each time we open a terminal
echo "source /opt/ros/noetic/setup.bash" >> /home/$USER/.bashrc
echo "source /home/$USER/catkin_ws/devel/setup.bash" >> /home/$USER/.bashrc

echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models" >> /home/$USER/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/$USER/catkin_ws/src/UNTITLED_UGV/ugv_sim/models" >> /home/$USER/.bashrc

