# iiwa_grasp

## Contents

### ROS package

- **flex_gazebo**: Gazebo simulation contains camera and marker

- **flex_gazebo_iiwa**: Gazebo simulation for the iiwa

- **flex_gazebo_interbotix**: Gazebo simulation for the interbotix manipulator

- **flex_grasp** :  Manipulator control to grasp food objects. Contains all the ROS nodes.

- **flex_sdh_moveit**: SDH files required for control via MoveIt!

- **rqt_user_interface**: graphical user interface

### Others

- **detect_truss**: Computer vision pipeline for Python (ROS independent)


## Install



### Setup
1.  Clone the master branch:
```
	git clone https://gitlab.tudelft.nl/pvkulkarni/flexcraft_jelle.git
```


### Run
1. To run in simulation we first launch the enviroment. To launch the interbotix enviroment launch:
    ```
    roslaunch flex_grasp interbotix_enviroment.launch
    ```
    To toggle between Gazebo simulation and real hardware use:
    - camera_sim: simulate the camera (default: true)
    - robot_sim: simulate the manipulator (default: true)
    For this walktrough I assume you are suing the simulation

2. Gazebo should start, by default it is paused (this behaviour can be chnaged in the launch files). Unpause the simulation by hitting play on the bar shown at the bottom

    ![Gazebo](images/gazebo.png)


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
- **Interbotix PincherX 150** (possibly all others from the interbotix series, but this has not been tested)


End-effector:

- **SDH**

Carmera:

- **Intel RealSense D435**
