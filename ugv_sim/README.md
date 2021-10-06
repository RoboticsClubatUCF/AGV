## ugv_sim: Ground Vehicle Simulation Package

A ROS package that contains the resources (models/scripts/worlds) to simulate various ground robots. Currently being used as part of the UNTITLED_UGV repo, but broadly useful.

Currently, this package is meant to target ROS Noetic/Ubuntu 20.04. 

### Setup/Configuration:

This package requires Ubuntu 20.04 (either as a VM, container, or actual system). Like all ROS1 packages, it's meant to be placed inside a [catkin_ws](http://wiki.ros.org/catkin/conceptual_overview) at path ```/home/$USER/catkin_ws```

1. Clone this repo into ```catkin_ws/src``` with:

   > git clone https://github.com/RoboticsClubatUCF/UNTITLED_UGV.git

   (NOTE: this will also pull down every other package in UNTITLED_UGV)

2. Make sure that our workspace builds with ```catkin_make```

   > cd ~/catkin_ws
   >
   > catkin_make

   This will build all of the packages inside our catkin workspace. Make sure that this completes without errors.

3.  Run the setup script to grab all of our dependencies (including ROS, if necessary) and configure our environment variables.

   > cd ~/catkin_ws/src/UNTITLED_UGV/ugv_sim/
   >
   > ./setup.sh

4. We should be all set. To test it, try to launch the simulator:

   > roslaunch ugv_sim sim.launch gazebo:=true

   If you see a Gazebo window with a robot in it, we're good to go.

### Anatomy

#### config/

The .yaml configuration files that determine the behavior of various ROS nodes. On launch, selected .yaml files will be parsed by ROS and converted into parameters stored on the shared [Parameter Server](http://wiki.ros.org/Parameter%20Server). These parameters can be retrieved/modified at runtime to adjust nodes or pass (static) information back and forth.

#### scripts/

Various ROS-adjacent scripts created for convenience when simulating robots, such as a keyboard teleoperation script (```motors.py```).

#### launch/

ROS [launch files](http://wiki.ros.org/roslaunch) to be invoked by ```roslaunch``` command to bring up our ROS nodes at system start. Each system supported by this package can have its own set of launch files, on top of those listed here.

#### models/

The models that will be loaded into Gazebo for simulation purposes. Currently, there is just ```bowser2``` in here, but we'll add more as time goes on.

#### meshes/

Meshes to be used by the models in models/. Pointed to by the ```GAZEBO_RESOURCE_PATH``` environment variable.

#### worlds/

.world files are the environment that our robot models get loaded into in the Gazebo simulator. World files can contain models themselves, as well as various configuration parameters about the environment, lighting, etc.

#### rviz/

.rviz files used for data visualization.

