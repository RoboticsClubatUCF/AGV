# simple_sim
### a ROS package that contains resources for a simple UGV simulation
---

## Setup
### **Dependencies/Requirements**
* Ubuntu 18.04 (supports ROS1 Melodic)
* ROS1 Melodic (ros-melodic-desktop-full)
* [Navigation Stack](https://github.com/ros-planning/navigation/tree/melodic-devel) (ros-melodic-navigation)
* [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/melodic-devel) (ros-melodic-slam-toolbox)
* [elevation_mapping](https://github.com/anybotics/elevation_mapping)
    * [grid_map](https://github.com/anybotics/grid_map) (ros-melodic-grid-map)
    * [kindr](https://github.com/ANYbotics/kindr.git)
    * [kindr_ros](https://github.com/ANYbotics/kindr_ros.git)
    * [perception_pcl](https://github.com/ros-perception/perception_pcl) (release 1.7.2)

### **Installation**

**Tldr: use the setup script to install ROS1 Melodic, set up workspace, and install dependencies -> `setup.sh`**

The long version:

This package is designed to be used with ROS1 Melodic (on Ubuntu 18.04) and cloned into a catkin workspace (like other ROS packages). You'll need to install ROS1 Melodic first. Alternatively, you can just follow the official guide [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

> sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

> sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

> sudo apt update

> sudo apt install ros-melodic-desktop-full

Set up our (bash) terminal to source our ROS install automatically
> echo "source /opt/ros/melodic/setup.bash" >> /home/$USER/.bashrc

Install some useful ROS dependencies
> sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y

> sudo apt-get install ros-melodic-catkin python-catkin-tools -y

> sudo rosdep init

> rosdep update

Once we have ROS1 Melodic installed and configured, we need to install the Navigation Stack and slam_toolbox:

> sudo apt install ros-melodic-navigation -y

> sudo apt install ros-melodic-slam-toolbox -y

Then, we'll create our workspace and clone this package into it: 

> mkdir -p ~/catkin_ws/src

> cd ~/catkin_ws/src

> git clone https://github.com/wesfletch/simple_sim.git --branch melodic

> cd ~/catkin_ws

<!-- > catkin_make -->

Finally, we need to install/build elevation_mapping. First, it's dependencies: 

* Grid Map:
    > sudo apt install ros-melodic-grid-map
* kindr:
    > git clone https://github.com/ANYbotics/kindr.git
* kindr_ros:
    > git clone https://github.com/ANYbotics/kindr_ros.git
* perception_pcl (we need release 1.7.2, the last one labeled for ROS1):
    > git clone https://github.com/ros-perception/perception_pcl --branch 1.7.2

And then elevation_mapping itself;
> git clone https://github.com/anybotics/elevation_mapping.git --branch release

Now we can build everything with `catkin build` (normally, it'd be catkin_make, but for whatever reason, elevation_mapping uses `build` and our workspace requires us to choose only one):
> catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

> catkin build

To verify everything is ready, run the following command to launch simple_sim (after sourcing your workspace again):

> roslaunch simple_sim elevation.launch
