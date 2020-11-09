### Contents

#### Nodes

- `analyze_point_cloud`: not used
- `calibrate`: generates the calibration poses, sends them to the move robot node and computing calibration
- `monitor robot`: is used to monitor the DYNAMIXELS of the interbotix robot, by reading temperature and error values. Furthermore it sets the PID values upon startup as defined in `/config/px150_pid`. This node does not do anything in simulation
- `move_gripper`: not used
- `move_robot`: takes commands from other nodes and moves the manipulater according to these commands
- 'object_detection': uses detect_truss to identify a valid grasp location
- `pick_place`: generates pick place commands and sends these to move_robot
- pipeline: contains the statemachine, commands all other nodes
- transform_pose: transforms a grasping pose as calculated by object_detection to a target pose for the manipulator
- visualize_object: not used

#### Classes
- communication: this class is used by many nodes to send commands to other nodes and wait for the result