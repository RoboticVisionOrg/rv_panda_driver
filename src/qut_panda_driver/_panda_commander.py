import rospy

from qut_manipulation_commander import ManipulationCommander

from std_msgs.msg import Int8
from franka_msgs.msg import FrankaState
from qut_manipulation_msgs.msg import MoveGripperActionResult

from _panda_moveit_commander import PandaMoveItCommander

class PandaCommander(ManipulationCommander):
  def __init__(self):

    self.move_group = rospy.get_param('~move_group', None)

    if not self.move_group:
      rospy.logerr('Unable to load move_group name from rosparam server path: move_group')
      sys.exit(1)

    ManipulationCommander.__init__(self, PandaMoveItCommander(self.move_group))

    self.recover_on_estop = rospy.get_param('/manipulation_commander/recover_on_estop', True)
    
    # handling e-stop
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.state_cb)
    self.estop_publisher = rospy.Publisher('estop', Int8, queue_size=1)
    self.last_estop_state = 0

  def state_cb(self, msg):
    out = Int8(0)

    if msg.robot_mode == FrankaState.ROBOT_MODE_USER_STOPPED:
      out.data = 1
    elif msg.robot_mode == FrankaState.ROBOT_MODE_IDLE:
      if self.recover_on_estop and self.last_estop_state == 1:
        self.moveit_commander.recover()

    self.estop_publisher.publish(out)
    self.last_estop_state = out.data

  def gripper_cb(self, goal):
    print(goal)
    self.moveit_commander.grasp(goal.width, goal.e_outer, goal.e_inner, goal.speed)
    self.gripper_server.set_succeeded(MoveGripperActionResult(result=0))
