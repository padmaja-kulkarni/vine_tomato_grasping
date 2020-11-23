# Taeke MSc
This repository contains all code used for my MSc thesis: "A geometry-based grasping method for vine tomato". Fot the thesis itself visit [this](https://surfdrive.surf.nl/files/index.php/s/StoH7xA87zUxl79) link. This repository can be used in two manners:

1. **Truss detection**: identify various truss features in a given image, only requires detect truss, for documnetation see [here](/detect_truss).
2. **Truss manipulation**: grasp vine tomato, requires all ros packages and the computer vision file, for further documentation see [here](/flex_grasp).


## Contents

### ROS packages

- **flex_gazebo**: Gazebo simulation contains camera and marker

- **flex_gazebo_iiwa**: Gazebo simulation for the iiwa

- **flex_gazebo_interbotix**: Gazebo simulation for the interbotix manipulator

- **flex_grasp** :  Manipulator control to grasp food objects. Contains all the ROS nodes.

- **flex_sdh_moveit**: SDH files required for control via MoveIt!

- **rqt_user_interface**: graphical user interface

### Others

- **detect_truss**: Computer vision pipeline for Python (ROS independent)
