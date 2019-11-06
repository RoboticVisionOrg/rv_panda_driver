# QUT Panda Driver

## Overview

The Panda Driver provides a series of components for initalising and controlling the Franka-Emika Panda robotic arm.

This package provides a simplified interface for controlling the arm in in a variety of articulation modes by extending the [QUT Manipulation Driver](https://bitbucket.org/acrv/rv_manipulation_driver) package.

## Usage

### Launching the driver

The Panda Driver can be launched by executing the following command:

```bash
roslaunch rv_panda_driver robot_bringup.launch
```

### Moving the arm

The Panda Driver provides a number of options for moving the robot arm. These include moving to fixed poses, as well as moving the arm by specifiying cartesian velocities for the end-effector.

#### Moving the End-Effector to an Arbitrary Pose

The following example moves the end-effector of the Panda robot to an arbitrary position and orientation w.r.t. to the robots base frame.
```
import rospy
import actionlib

from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from geometry_msgs.msg import PoseStamped

# initialise ros node
rospy.init_node('move_to_points_example')

# Create a ros action client to communicate with the controller
client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
client.wait_for_server()

# Create a target pose
target = PoseStamped()
target.header.frame_id = 'panda_link0'

# Populate with target position/orientation (READY POSE)
target.pose.position.x = 0.307
target.pose.position.y = 0.000
target.pose.position.z = 0.590

target.pose.orientation.x = -1.00
target.pose.orientation.y =  0.00
target.pose.orientation.z =  0.00
target.pose.orientation.w =  0.00

# Create goal from target pose
goal = MoveToPoseGoal(goal_pose=target)

# Send goal and wait for it to finish
client.send_goal(goal)
client.wait_for_result()
```

#### Moving the End-Effector in Cartesian Space

The following example moves the arm at 2cm a second in the z-axis of the base-frame for 5 seconds. For safety reasons the driver has an expected minimum frequency of 100Hz.

```
import rospy
import timeit
from geometry_msgs.msg import TwistStamped

# initialise ros node
rospy.init_node('cartesian_motion')

# Create the publisher (queue size tells ROS to only publish the latest message)
publisher = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)

# Create an initial start time
start = timeit.default_timer()

# Create a velocity message that will instruct the robot to move at 2cm a second in the z-axis of the base frame.
velocity = TwistStamped()
velocity.twist.linear.z = 0.02

# Publish the velocity message to the panda driver at a frequency of 100Hz
while (timeit.default_timer() - start) < 5:
  publisher.publish(velocity)
  rospy.sleep(0.01)
  
# Publish an empty TwistStamped to ensure that the arm stops moving
publisher.publish(TwistStamped())
```

### Subscribed Topics

- **/arm/cartesian/velocity** ([geometry_msgs/TwistStamped](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
Moves the end-effector in cartesian space w.r.t. the target frame_id (base frame if no frame_id is set).

### Services

- **/arm/home** ([std_srvs/Empty](http://docs.ros.org/jade/api/std_srvs/html/srv/Empty.html))
Moves the robot back to its initial ready pose.

### Action API

#### Pose Control

- **/arm/cartesian/pose/goal** ([rv_msgs/MoveToPoseGoal](https://bitbucket.org/acrv/rv_msgs/src/master/action/MoveToPose.action))
Moves the end-effector to the requested goal pose w.r.t. the base frame.

- **/arm/cartesian/pose/cancel** ([actionlib_msgs/GoalID](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))
Cancels the currently executing goal.

- **/arm/cartesian/pose/feedback** ([rv_msgs/MoveToPoseGoal](https://bitbucket.org/acrv/rv_msgs/src/master/action/MoveToPose.action))
Feedback from the currently executing goal.

- **/arm/cartesian/pose/status** ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html))
Status information on goals sent to the driver.

- **/arm/cartesian/pose/result** ([rv_msgs/MoveToPoseGoal](https://bitbucket.org/acrv/rv_msgs/src/master/action/MoveToPose.action))
The result of the pose goal request.

#### Named Pose Control

- **/arm/cartesian/named_pose/goal** ([rv_msgs/MoveToNamedPoseGoal](https://bitbucket.org/acrv/rv_msgs/src/master/action/MoveToNamedPose.action))
Moves the end-effector to a pre-defined joint configuration.

- **/arm/cartesian/named_pose/cancel** ([actionlib_msgs/GoalID](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))
Cancels the currently executing goal.

- **/arm/cartesian/named_pose/feedback** ([rv_msgs/MoveToNamedPoseFeedback]((https://bitbucket.org/acrv/rv_msgs/src/master/action/MoveToNamedPose.action)))
Feedback from the currently executing goal.

- **/arm/cartesian/named_pose/status** ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html))
Status information on goals sent to the driver.

- **/arm/cartesian/named_pose/result** ([rv_msgs/MoveToNamedPoseResult]((https://bitbucket.org/acrv/rv_msgs/src/master/action/MoveToNamedPose.action)))
The result of the goal request.
