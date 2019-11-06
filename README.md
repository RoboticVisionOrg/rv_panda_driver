# QUT Panda Driver

## Overview

The Panda Driver provides a series of components for initalising and controlling the Franka-Emika Panda robotic arm.

This package provides a simplified interface for controlling the arm in in a variety of articulation modes by extending the [RV Manipulation Driver](https://bitbucket.org/acrv/rv_manipulation_driver) package.

![System Diagram] (docs/Panda.png)

## Usage

### Launching the driver

The Panda Driver can be launched by executing the following command:

```bash
roslaunch rv_panda_driver robot_bringup.launch
```

Other things to note when starting up the Panda:

* The lights indicate the status of the robot. 
    * Blue = happy and ready to move 
    * White = happy but estopped
    * Yellow = error
* You will need to release the joint locks through the Franka interface
* When operating the arm have the E-stop handy for safety at all times
* When the arm is E-stopped it can be moved freely using the wrist switch 

To ensure the arm is working once started you can call the home routine. 
```bash
rosservice call /arm/home
```

### Moving the arm

The Panda Driver provides a number of options for moving the robot arm. These include moving to fixed poses, as well as moving the arm by specifiying cartesian velocities for the end-effector.

#### Moving the End-Effector to an Arbitrary Pose

The following code example moves the end-effector of the Panda robot to an arbitrary position and orientation w.r.t. to the robots base frame.
```
import rospy
import actionlib

from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from geometry_msgs.msg import PoseStamped

# initialise ros node
rospy.init_node('move_to_points_example')

# Create a ros action client to communicate with the driver
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

The following code example moves the arm at 2cm a second in the z-axis of the base-frame for 5 seconds. For safety reasons the driver has an expected minimum frequency of 100Hz.

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

### Managing Named Poses
While the MoveIt framework provide the ability to store joint configurations as named poses, these must be defined in the MoveIt configuration of the robot and cannot be adjusted during operation. 

The panda driver solves this issue by providing the ability to save and remember joint configurations of the Panda arm for future, while also providing access to named poses provided by moveit.

#### Saving Named Poses
The following command demonstrates how to the current joint configuration of the robot with the name *test_pose*:

```
rosservice call /arm/set_named_pose "pose_name: 'test_pose' overwrite: false"
```

#### Getting Named Poses
The following command demonstrates how to get the current set of stored named poses (created either through the driver or moveit)
```
rosservice call /arm/get_named_poses
```

#### Moving to a Named Pose
The following code example demonstrates how to move to a named pose:
```
import rospy
import actionlib

from rv_msgs.msg import MoveToNamedPoseAction, MoveToNamedPoseGoal

# initialise ros node
rospy.init_node('named_pose_example')

# Create a ros action client to communicate with the driver
client = actionlib.SimpleActionClient('/arm/cartesian/named_pose', MoveToNamedPoseAction)
client.wait_for_server()

# Create and send a goal to move to the named pose test_pose
client.send_goal(MoveToNamedPoseGoal(pose_name='test_pose'))
client.wait_for_result()
```

**Note:** The goal pose can be a named pose created either by the driver or moveit. However, the named poses provided by the driver will take priority in the event of a naming conflict.

### Subscribed Topics

- **/arm/cartesian/velocity** ([geometry_msgs/TwistStamped](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
Moves the end-effector in cartesian space w.r.t. the target frame_id (base frame if no frame_id is set).

### Publish Topics

- **/arm/state**  ([rv_msgs/ManipulatorState](https://bitbucket.org/acrv/rv_msgs/src/master/msg/ManipulatorState.msg))
Provides information on the current state of the manipulator including the pose of the end-effector w.r.t. to the base link, whether the manipulator is experiencing a cartesian contact and collision as a bit-wised error state flag.

### Services

- **/arm/home** ([std_srvs/Empty](http://docs.ros.org/jade/api/std_srvs/html/srv/Empty.html))
Moves the robot back to its initial ready pose.

- **/arm/recover** ([std_srvs/Empty](http://docs.ros.org/jade/api/std_srvs/html/srv/Empty.html))
Recovers from collision or limit violation error states that will put the robot into a non-operable state.

- **/arm/stop** ([std_srvs/Empty](http://docs.ros.org/jade/api/std_srvs/html/srv/Empty.html))
Stops the current motion of the current.

- **/arm/get_named_poses** (rv_msgs/GetNamesList](https://bitbucket.org/acrv/rv_msgs/src/master/srv/GetNamesList.srv))
Gets a list of currently stored named poses (includes both moveit and driver stored named poses).

- **/arm/set_named_pose** ([rv_msgs/SetNamedPose](https://bitbucket.org/acrv/rv_msgs/src/master/srv/SetNamedPose.srv))
Saves the current joint configuration of the panda with the provided pose name.

- **/arm/set_cartesian_impedance** ([rv_msgs/SetCartesianImpedance](https://bitbucket.org/acrv/rv_msgs/src/master/srv/SetCartesianImpedance.srv)
Adjusts the impedenace of the end-effector position in cartesian space.

- **/arm/get_link_position** ([rv_msgs/GetRelativePose](https://bitbucket.org/acrv/rv_msgs/src/master/srv/GetRelativePose.srv))
A convenience wrapper around the ROS transform lookup service that provides the relative pose of a target frame w.r.t. a reference frame.

### Action API

#### Pose Control

- **/arm/cartesian/pose** ([rv_msgs/MoveToPose.action](https://bitbucket.org/acrv/rv_msgs/src/master/action/MoveToPose.action))
Moves the end-effector to the requested goal pose w.r.t. the base frame.


#### Named Pose Control

- **/arm/cartesian/named_pose** ([rv_msgs/MoveToNamedPose.action](https://bitbucket.org/acrv/rv_msgs/src/master/action/MoveToNamedPose.action))
Moves the end-effector to a pre-defined joint configuration.

#### Gripper

- **/arm/gripper** ([rv_msgs/ActuateGripper.action](https://bitbucket.org/acrv/rv_msgs/src/master/action/ActuateGripper.action))
Actuates the gripper based on the requested mode. The static mode will move the gripper to the requested width. The grasp mode will attempt to grasp an object of width plus/minus a tolernace factor.
