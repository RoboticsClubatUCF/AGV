# ugv_description 

Contains simulation models for our robot.

**Docs:**
- URDF: wiki.ros.org/urdf
- xacro package: http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File
	
### model.urdf.xacro
This is the main definition file. It contains physical parameters of the robot, the robots "links," and the relationship between them. If there are PHYSICAL changes to the robot, this is the file to edit. 

### model.macro
Convenience macros used to avoid replicating effort and clean up model.urdf.xacro.

### model.gazebo
The GAZEBO-SPECIFIC parameters of the robot, including Gazebo physical properties (contained within <gazebo> tags) and simulation plugins. The stuff in this file affects the Gazebo simulator ONLY.

### model.urdf
This is the auto-generated simulation model description, created by concatenating model.macro and model.gazebo to model.urdf.xacro, and then xacro expanding the result of that concatenation. Since this is auto-generated at model load-time, DO NOT MODIFY BY HAND. Your changes WILL be lost the next time you load the model. Edit the appropriate sub-file.

### model.config
Just a description of the model. Details here will affect the name of the model that is loaded into gazebo, but otherwise don't matter at all.
