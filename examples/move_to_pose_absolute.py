import rospy
import actionlib

from qut_manipulation_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from geometry_msgs.msg import PoseStamped

# initialise ros node
rospy.init_node('move_to_points_example')

# Create a ros action client to communicate with the controller
client = actionlib.SimpleActionClient('/cartesian/pose', MoveToPoseAction)
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
goal = MoveToPoseGoal(pose=target)

# Send goal and wait for it to finish
client.send_goal(goal)
client.wait_for_result()

# Adjust to move 10cm above ready pose and create new goal
target.pose.position.z += 0.1
goal = MoveToPoseGoal(pose=target)

# Seng goal and wait for it to finish
client.send_goal(goal)
client.wait_for_result()


# Move back to original target position
target.pose.position.z -= 0.1
goal = MoveToPoseGoal(pose=target)

# Seng goal and wait for it to finish
client.send_goal(goal)
client.wait_for_result()
