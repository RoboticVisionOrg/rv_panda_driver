import rospy
import actionlib
import franka_gripper.msg

from rv_manipulation_commander import ManipulationMoveItCommander
from franka_control.msg import ErrorRecoveryActionGoal


class PandaMoveItCommander(ManipulationMoveItCommander):
  def __init__(self, group_name=None):
    ManipulationMoveItCommander.__init__(self, group_name)
    self.reset_publisher = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=1)

  def home_gripper(self):
      client = actionlib.SimpleActionClient('franka_gripper/homing', franka_gripper.msg.HomingAction)
      client.wait_for_server()
      client.send_goal(franka_gripper.msg.HomingGoal())
      return client.wait_for_result()

  def set_gripper(self, width, speed=0.1, wait=True):
      client = actionlib.SimpleActionClient('franka_gripper/move', franka_gripper.msg.MoveAction)
      client.wait_for_server()
      client.send_goal(franka_gripper.msg.MoveGoal(width, speed))
      if wait:
          return client.wait_for_result()
      else:
          return True

  def grasp(self, width=0, e_inner=0.1, e_outer=0.1, speed=0.1, force=1):
      client = actionlib.SimpleActionClient('franka_gripper/grasp', franka_gripper.msg.GraspAction)
      client.wait_for_server()
      client.send_goal(
          franka_gripper.msg.GraspGoal(
              width,
              franka_gripper.msg.GraspEpsilon(e_inner, e_outer),
              speed,
              force
          )
      )
      return client.wait_for_result()

  def recover(self):
      self.reset_publisher.publish(ErrorRecoveryActionGoal())
      rospy.sleep(3.0)
