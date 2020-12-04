### Contents


### Usage
Launch a visualization of the px150 with the custom gripper as followes:
```
roslaunch interbotix_descriptions description.launch jnt_pub_gui:=true use_default_gripper_fingers:=false external_urdf_loc:='$(find flex_gazebo_interbotix)/urdf/custom_gripper.urdf.xacro'
```
