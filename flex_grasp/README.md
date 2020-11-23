# Flex Grasp
A ROS packages for munipulating vine tomato

## Install

### Install ROS
Install ROS Melodic. Make sure sure that you have your environment properly setup, and that you have the most up to date packages:
```
rosdep update  # No sudo
sudo apt-get update
sudo apt-get dist-upgrade
```

### Create A Workspace
You will need to have a ROS workspace setup:
```
mkdir -p ~/flexcraft_ws/src
cd ~/flexcraft_ws/
catkin_make
```

### Download the source code
clone this repository
```
git clone https://github.com/padmaja-kulkarni/taeke_msc.git
```

### Basic Dependencies
Install dependencies
```
rosdep install --from-paths src --ignore-src -r -y
```


### Other Dependencies
Some more packages need to be installed. For Interbotix support we require interbotix_ros_arms:
```
cd ~/flexcraft_ws/src
git clone --single-branch --branch reboot_service https://github.com/TaekedeHaan/interbotix_ros_arms.git
```
This forked repository has an additional rebood service which is automatically called when a motor reports an Harware Error. Note that the used repository is now in legacy mode, the [updated sdk](https://github.com/Interbotix/interbotix_ros_core) contains this reboot serive by default.

For iiwa support we require iiwa_stack:
```
cd ~/flexcraft_ws/src
git clone https://github.com/IFL-CAMP/iiwa_stack
```

For calibration easy_handeeye is used:
```
cd ~/flexcraft_ws/src
git clone https://github.com/IFL-CAMP/easy_handeye.git
```

For realsense camera support realsense-ros is used
```
cd ~/flexcraft_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
```

For fine-tuning parameters of the computer vision pipeline rqt_ez_publisher is used:
```
cd ~/flexcraft_ws/src
git clone --single-branch --branch initialize-subscribe https://github.com/TaekedeHaan/rqt_ez_publisher.git
```
This forks contains some modifications to initialize the parameters in the GUI to the values last published. Note that you can also use the default library, howver this initialization makes life a bit easier.


## Run (Simulation)
1. To run in simulation we first launch the enviroment. To launch the interbotix enviroment run in your terminal:
    ```
    roslaunch flex_grasp interbotix_enviroment.launch
    ```

2. Gazebo should start by default it is paused (this behaviour can be chnaged in the launch files).

    ![Gazebo](images/gazebo.png)

3. Unpause the simulation by hitting play on the bar shown at the bottom, RViz should start
    
    ![Gazebo](images/rviz.png)
    
4. You have succesfully started the enviroment. To stat the controls run in your terminal:
    ```
    roslaunch flex_grasp interbotix_control.launch
    ``` 
5. An rqt graphical user interface should pop up, sometimes in initializes incorrect, if this happens hit Ctrl + C, and retry

    ![Gazebo](images/rqt.png)

6. You have succesfully initialized the controls, and the virtual robot is ready to go.


### Run (Real Hardware)
1. Again, first launch the enviroment. To launch the interbotix enviroment for real hardware run in your terminal:
    ```
    roslaunch flex_grasp interbotix_enviroment.launch camera_sim:=false robot_sim:=false
    ```
    Here we use the parameters to toggle between Gazebo simulation and real hardware:
    - camera_sim: simulate the camera (default: true)
    - robot_sim: simulate the manipulator (default: true)
    
2. You have succesfully started the enviroment. To stat the controls run in your terminal:
    ```
    roslaunch flex_grasp interbotix_control.launch
    ``` 
3. An rqt graphical user interface should pop up, sometimes in initializes incorrect, if this happens hit Ctrl + C, and retry

4. You have succesfully initialized the controls, and the robot is ready to go.

### Command (Virtual) Robot
To activate an action, a command needs to be published on the `ROBOT_NAME/pipeline_command`. This can be done using the GUI:
- Sleep: command the manipulator to the resting pose
- Home: command the manipulator to the upright pose
- Ready: command the robot to the initial pose
- Open: command the end effector to open
- Close: command the end effector to close
- Calibrate: determine the pose between the robot base and camera
- Detect Truss: command to computer vision pipeline to detect the truss
- Save Image: sace the current image
- Pick and Plance: execute a pick and place routing
- Experiment: Repeatedly execute Detect Truss, Save Image and Pick and Place (easy for conducitng experiments)

With the drup down menu you can select where to store the results.


## Supported hardware

Manipulator:

- **Interbotix PincherX 150** (possibly all others from the interbotix series, but this has not been tested)
- **KUKA LBR IIWA 7** (deprecated in simulation + not tested on actual hardware)

End-effector:

- **SDH** (deprecated in simulation + not tested on actual hardware)

Carmera:

- **Intel RealSense D435**


### Contents

#### Nodes

- `analyze_point_cloud`: not used
- `calibrate`: generates the calibration poses, sends them to the move robot node and computing calibration
- `monitor robot`: is used to monitor the DYNAMIXELS of the interbotix robot, by reading temperature and error values. Furthermore it sets the PID values upon startup as defined in `/config/px150_pid`. This node does not do anything in simulation
- `move_gripper`: not used
- `move_robot`: takes commands from other nodes and moves the manipulater according to these commands
- `object_detection`: uses detect_truss to identify a valid grasp location
- `pick_place`: generates pick place commands and sends these to move_robot
- `pipeline`: contains the statemachine, commands all other nodes
- `transform_pose`: transforms a grasping pose as calculated by object_detection to a target pose for the manipulator
- `visualize_object`: not used

#### Classes
- `communication`: this class is used by many nodes to send commands to other nodes and wait for the result

#### Messages
- `ImageProcessingSettings`
- `Peduncle`
- `Tomato`
- `Truss`


#### Enums
To store the state of different parts of the system, enums are used. These are defined in the messa files.
- `DynamixelErrorCodes`
- `FlexGraspCommandCodes`
- `FlexGraspErrorCodes`

### Info

All nodes run in the `robot_name` namespace to allow for multiple robots present
