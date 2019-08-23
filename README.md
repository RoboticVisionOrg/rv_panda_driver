# QUT Panda Driver

## Overview

The Panda Driver provides a series of components for initalising and controlling the Franka-Emika Panda robotic arm.

This package provides a simplified interface for controlling the arm in in a variety of articulation modes by extending the [QUT Manipulation Commander](https://bitbucket.org/acrv/qut_manipulation_commander) package.

## Usage

### Launching the driver

The Panda Driver can be launched by executing the following command:

```bash
roslaunch qut_panda_driver robot_bringup.launch
```

### Subscribed Topics

- **cartesian/velocity** ([geometry_msgs/Twist](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
Moves the end-effector in cartesian space w.r.t. the base frame.

### Services

- **home** ([std_srvs/Empty](http://docs.ros.org/jade/api/std_srvs/html/srv/Empty.html))
Moves the robot back to its initial ready pose.

### Action API

#### Pose Control

- **cartesian/pose/goal** ([qut_manipulation_msgs/MoveGripperToPoseGoal](https://bitbucket.org/acrv/qut_manipulation_msgs/src/master/action/MoveGripperToPose.action))
Moves the end-effector to the requested goal pose w.r.t. the base frame.

- **cartesian/pose/cancel** ([actionlib_msgs/GoalID](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))
Cancels the currently executing goal.

- **cartesian/pose/feedback** ([qut_manipulation_msgs/MoveGripperToPoseGoal](https://bitbucket.org/acrv/qut_manipulation_msgs/src/master/action/MoveGripperToPose.action))
Feedback from the currently executing goal.

- **cartesian/pose/status** ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html))
Status information on goals sent to the driver.

- **cartesian/pose/result** ([qut_manipulation_msgs/MoveGripperToPoseGoal](https://bitbucket.org/acrv/qut_manipulation_msgs/src/master/action/MoveGripperToPose.action))
The result of the pose goal request.

#### Named Pose Control

- **cartesian/named_pose/goal** ([qut_manipulation_msgs/MoveGripperToNamedPoseGoal](https://bitbucket.org/acrv/qut_manipulation_msgs/src/master/action/MoveGripperToNamedPose.action))
Moves the end-effector to a pre-defined joint configuration.

- **cartesian/named_pose/cancel** ([actionlib_msgs/GoalID](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))
Cancels the currently executing goal.

- **cartesian/named_pose/feedback** ([qut_manipulation_msgs/MoveGripperToNamedPoseFeedback]((https://bitbucket.org/acrv/qut_manipulation_msgs/src/master/action/MoveGripperToNamedPose.action)))
Feedback from the currently executing goal.

- **cartesian/named_pose/status** ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html))
Status information on goals sent to the driver.

- **cartesian/named_pose/result** ([qut_manipulation_msgs/MoveGripperToNamedPoseResult]((https://bitbucket.org/acrv/qut_manipulation_msgs/src/master/action/MoveGripperToNamedPose.action)))
The result of the goal request.
