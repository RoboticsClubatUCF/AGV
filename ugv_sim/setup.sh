# if ROS isn't installed, install it
if ! [ -d "/opt/ros/noetic" ]
then
	echo "ROS not found, installing ROS Noetic"
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	
	sudo apt install c-y url # if you haven't already installed curl
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	
	sudo apt update
	sudo apt install -y ros-noetic-desktop-full # this takes a while
fi

source /opt/ros/noetic/setup.bash

# some ROS-related tools we may need
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# initialize rosdep
sudo rosdep init
rosdep update

# some miscellaneous tools
sudo apt install -y git
sudo apt install -y python-3 pip
pip3 install pynput

# ugv_sim dependencies:
sudo apt install -y ros-noetic-navigation
sudo apt install -y ros-noetic-slam-toolbox

# if we somehow don't have a catkin_ws, we're gonna make one and clone our code into it
if ! [-d "/home/$USER/catkin_ws"]
then
	echo "Creating catkin workspace at: /home/$USER/catkin_ws"
	mkdir -p /home/$USER/catkin_ws/src
	cd /home/$USER/catkin_ws/src
	git clone https://github.com/RoboticsClubatUCF/UNTITLED_UGV.git
	cd /home/$USER/catkin_ws
	catkin_make # catkin_make to create our build and devel folders
fi

source /home/$USER/catkin_ws/devel/setup.bash

# Adding environment variables to .bashrc means we don't need to source them each time we open a terminal

bashrc=/home/$USER/.bashrc

# if we don't already have the ros-distro source command in .bashrc, put it there
if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" $bashrc ; then
	echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $bashrc
fi
#  source our catkin_ws, if we don't already
if ! grep -q "source /home/$USER/catkin_ws/devel/setup.bash" $bashrc; then	
	echo "source /home/$USER/catkin_ws/devel/setup.bash" >> /home/$USER/.bashrc
fi

echo "# sourcing our Gazebo setup files and environment variables" $bashrc
echo "# this is how Gazebo actually finds our models" >> $bashrc

if ! grep -q "source /usr/share/gazebo-11/setup.sh"; then
	echo "source /usr/share/gazebo-11/setup.sh" >> $bashrc
fi

if ! grep -q "GAZEBO_MODEL_PATH=" $bashrc; then
	echo "GAZEBO_MODEL_PATH not found, setting GAZEBO_MODEL_PATH"
	echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models" >> $bashrc
	echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/$USER/catkin_ws/src/UNTITLED_UGV/ugv_sim/models" >> $bashrc
fi

if ! grep -q "GAZEBO_RESOURCE_PATH=" $bashrc; then
	echo "GAZEBO_RESOURCE PATH not found, setting GAZEBO_RESOURCE_PATH"
	echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/$USER/catkin_ws/src/UNTITLED_UGV/ugv_sim/meshes" >> $bashrc
fi

source $bashrc

echo "Setup complete. Get to work, nerd."`

