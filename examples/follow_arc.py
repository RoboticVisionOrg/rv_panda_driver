
import sys
import math
import numpy as np
import quaternion
import transforms3d as t3

def vector_angle(a, b):
  return np.arccos(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b)))

def plot_arc(center, start_pose, end_pose, n):
  u = start_pose[0]
  v = end_pose[0]

  start_wxyz = start_pose[1]
  end_wxyz = end_pose[1]

  a = vector_angle(u, v)

  poses = []

  for k in range(n+1):
    theta = k*a/n
    offset = np.sin(a-theta)*u + np.sin(theta)*v*np.sin(a)
    
    wxyz = quaternion.slerp(start_wxyz, end_wxyz, 0, 1, (1/float(n))*k)
    xyzw = [wxyz.x, wxyz.y, wxyz.z, wxyz.w]

    poses.append((center + offset, xyzw))

  return poses

def move_to(position, quaternion):
  # Create a target pose
  target = PoseStamped()
  target.header.frame_id = 'panda_link0'

  # Populate with target position/orientation (READY POSE)
  target.pose.position.x = position[0]
  target.pose.position.y = position[1]
  target.pose.position.z = position[2]

  target.pose.orientation.x = quaternion[0]
  target.pose.orientation.y = quaternion[1]
  target.pose.orientation.z = quaternion[2]
  target.pose.orientation.w = quaternion[3]

  # Create goal from target pose
  goal = MoveToPoseGoal(goal_pose=target)

  # Send goal and wait for it to finish
  client.send_goal(goal)
  client.wait_for_result()

import rospy
import actionlib

from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from rv_msgs.srv import GetRelativePose

from geometry_msgs.msg import PoseStamped

# initialise ros node
rospy.init_node('arc_follow_example')

center = (0.35, 0.0, 0.2)

start_pose = (np.array([0, 0.400, 0]), quaternion.quaternion(0.5, 0.5, -0.5, 0.5))
end_pose = (np.array([0, 0, 0.400]), quaternion.quaternion(0.0, 0.70710678, -0.70710678, 0.0))

n = 10

# Create a ros action client to communicate with the controller
client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
client.wait_for_server()

for pose in plot_arc(center, start_pose, end_pose, n):
  position = pose[0]
  xyzw = pose[1]

  print(position, xyzw)
  raw_input()
  
  move_to(position=position, quaternion=xyzw)
  
