import rospy
import actionlib
import franka_gripper.msg

from rv_manipulation_driver import ManipulationMoveItDriver
from franka_control.msg import ErrorRecoveryAction, ErrorRecoveryGoal

class PandaMoveItCommander(ManipulationMoveItDriver):
  def __init__(self, group_name=None):
    ManipulationMoveItDriver.__init__(self, group_name)
    self.reset_client = actionlib.SimpleActionClient('/franka_control/error_recovery', ErrorRecoveryAction)

  def home_gripper(self):
    client = actionlib.SimpleActionClient('franka_gripper/homing', franka_gripper.msg.HomingAction)
    client.wait_for_server()
    client.send_goal(franka_gripper.msg.HomingGoal())
    return client.wait_for_result()

  def set_gripper(self, width, speed=0.1):
    client = actionlib.SimpleActionClient('franka_gripper/move', franka_gripper.msg.MoveAction)
    client.wait_for_server()
    print(width, speed if speed > 0 else 0.1)
    client.send_goal(franka_gripper.msg.MoveGoal(max(min(width, 0.075), 0.0), speed if speed > 0 else 0.1))
    print('Waiting for move result')
    result = client.wait_for_result(timeout=rospy.Duration(5))
    print('Received result:', result)
    return result

  def grasp(self, width=0, e_inner=0.1, e_outer=0.1, speed=0.1, force=1):
    client = actionlib.SimpleActionClient('franka_gripper/grasp', franka_gripper.msg.GraspAction)
    client.wait_for_server()
    client.send_goal(
      franka_gripper.msg.GraspGoal(
        width,
        franka_gripper.msg.GraspEpsilon(e_inner if e_inner >= 0.1 else 0.1, e_outer if e_inner >= 0.1 else 0.1),
        speed if speed > 0 else 0.1,
        force if force > 0 else 1
      )
    )
    print('Waiting for grasp result')
    result = client.wait_for_result(timeout=rospy.Duration(5))
    print('Received result:', result)
    return result

  def recover(self):
    self.reset_client.send_goal(ErrorRecoveryGoal())
    self.reset_client.wait_for_result()
      
