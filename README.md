# Taeke MSc
This repository contains all code used for my MSc thesis: "A geometry-based grasping method for vine tomato". Fot the thesis itself visit [this](https://surfdrive.surf.nl/files/index.php/s/StoH7xA87zUxl79) link. This repository can be used in two manners:

1. **Truss detection**: extract truss features from a given image and determined an optimal grasping pose. This is done in the ROS independent `flex_vision` package, for documentation see [here](/detect_truss).
2. **Truss manipulation**: Manipulator control to grasp vine tomatoes. This is done collabarativly with all ROS packages, for further documentation see [here](/flex_grasp).


## Contents

### ROS packages

- **flex_gazebo**: Gazebo simulation, containing general files such as camera and marker.

- **flex_gazebo_iiwa**: Gazebo simulation, containing files specifically for the iiwa manipulator.

- **flex_gazebo_interbotix**: Gazebo simulation, containing files specifically for the interbotix manipulator.

- **flex_grasp**: Manipulator control to grasp vine tomatoes. Contains all the ROS nodes.

- **flex_sdh_moveit**: SDH files required for control via MoveIt!

- **rqt_user_interface**: graphical user interface.

- **flex_calibrate**: Manipulator calibration

- **flex_shared_resources**: shared functions and classes

### Others

- **flex_vision**: Computer vision pipeline for Python (ROS independent)
