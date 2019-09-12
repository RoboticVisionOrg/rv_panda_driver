import rospy
import actionlib

from qut_manipulation_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from qut_manipulation_msgs.srv import LinkPose

from geometry_msgs.msg import PoseStamped

# initialise ros node
rospy.init_node('move_to_points_example')

# Create a ros action client to communicate with the controller
client = actionlib.SimpleActionClient('/cartesian/pose', MoveToPoseAction)
client.wait_for_server()

# Create a ros service client
get_pose = rospy.ServiceProxy('/get_link_position', LinkPose)
get_pose.wait_for_service()

# Get the current position of panda hand w.r.t. the panda base
current = get_pose('panda_hand', 'panda_link0')
print(current)
# Create a target pose based on our current position but moving up 10cm on the z axis
target = current.pose
target.pose.position.z += 0.1
print(target)
# Adjust target to move 10cm above ready pose and create new goal
goal = MoveToPoseGoal(pose=target)

# Seng goal and wait for it to finish
client.send_goal(goal)
client.wait_for_result()

# Adjust target to original position and create new goal
target = current.pose
target.pose.position.z -= 0.1

# Seng goal and wait for it to finish
client.send_goal(goal)
client.wait_for_result()
