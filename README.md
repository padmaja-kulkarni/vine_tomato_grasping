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
2. For some p[art we still rely on the iiwa launch files, therefore some minor chnages are required. Go to the file `/iiwa_stack/iiwa_moveit/launch/planning_context.launch`

3. Change line 15 from:
```
<include file="$(find iiwa_description)/launch/iiwa7_upload.launch">
```
To:
```
<include file="$(find flex_gazebo)/launch/upload.launch">
```
4. To make the Intel RealSense camera behave similar as the actual hardware we have to change the default resultution, open the file `/realsense_gazebo_plugin/models/realsense_camera/model.sdf`
5. For the depth and color sensor change the `width` and `height` values from 640 and 480 to 1920 and 1080 correspondingly.
6. For gazebo to be able to load the tomato texture go to `usr/share/gazebo-7/setup.sh` and add to the `GEZEBO_RESOURCE_PATH` variable the path to the `flex_grasp` package, in my case `~/catkin_ws/src/flexcraft_jelle/flex_grasp`
You are ready to go

### Run
1. To run in simulation we first launch the enviroment
```
roslaunch flex_grasp enviroment.launch
```
2. RViz en Gazebo should start, in RViz we still have to load the scene manually as follows

	1. in the MotionPlanning too go to the tab Scene Objects
	2. Select Inport From Text
	3. Navigate to the folder /flexcraft_jelle/flex_gazebo/worlds/ and select the file iiwa.scene
	4. select Publish Scene

3. Now run the controls
```
roslaunch flex_grasp sub.launch
```
If the following warning appears "Table and wall object not present, refusing to continue before they are added" the scene is not correctly loaded and/or published in RViz.

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
