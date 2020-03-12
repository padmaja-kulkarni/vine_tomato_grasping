# iiwa_grasp

## Contents

- **grasp_crop** :  Ros package for controlling a manipulator to grasp food objects. Contains all the ROS nodes.

- **detect_crop** :  Ros package for controlling the Kuka iiwa arm to grasp food objects.

## Install

### Prerequisit

- [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack): for controlling the manipulator.
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros): for using Intel RealSense cameras.
- [realsense_gazebo_plugin](https://github.com/SyrianSpock/realsense_gazebo_plugin): for simulating the realse camera in Gazebo.

### Setup
1.  Clone the master branch:
```
	git clone https://gitlab.tudelft.nl/pvkulkarni/flexcraft_jelle.git
```
2. To launch the modified world, and not the default iiwa world open the the following file  `/iiwa_stack/iiwa_gazebo/launch/iiwa_world.launch`

3. Change the following line:
```
<arg name="world_name" value="$(find iiwa_gazebo)/worlds/iiwa.world"/>
```
To:
```
<arg name="world_name" value="$(find iiwa_grasp)/worlds/iiwa.world"/>
```
4. To make the Intel RealSense camera behave similar as the actual hardware we have to change the default resultution, open the file `/realsense_gazebo_plugin/models/realsense_camera/model.sdf`
5. For all four sensros present change the `width` and `height` values from 640 and 480 to 1920 and 1080 correspondingly.

You are ready to go

### Run
1. To run in simulation we first launch the enviroment
```
roslaunch iiwa_moveit moveit_planning_execution.launch
```
2. RViz en Gazebo should start, in RViz we still have to load the scene manually as follows

	1. in the MotionPlanning too go to the tab Scene Objects
	2. Select Inport From Text
	3. Navigate to the folder /flexcraft_jelle/iiwa_grasp/worlds/ and select the file iiwa.scene
	4. select Publish Scene

3. Now run the controls
```
roslaunch iiwa_grasp sub.launch
```
If the following warning appears " Table and wall object not present, refusing to continue before they are added" the scene is not correctly loaded and/or published in RViz.

### Command
To activate an action, a message needs to be published on the `/iiwa/pipelineState`

```
rostopic pub -1 /iiwa/pipelineState std_msgs/String COMMAND
```

where COMMAND can be either `"DETECT"` or `"HOME"`.

## Supported hardware

Manipulator:

- **KUKA LBR IIWA 7**

End-effector:

- **-**

Carmera:

- **Intel RealSense D435**
