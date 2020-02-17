import rospy
import sys
import time

from rv_manipulation_driver import ManipulationDriver
from rv_manipulation_driver import transforms

from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

from rv_msgs.msg import JointVelocity
from rv_msgs.msg import ManipulatorState
from rv_msgs.msg import ActuateGripperAction, ActuateGripperActionResult
from rv_msgs.srv import SetCartesianImpedanceResponse

from franka_msgs.srv import SetCartesianImpedance as FrankaSetCartesianImpedance
from franka_msgs.srv import SetEEFrame, SetEEFrameRequest
from franka_msgs.msg import FrankaState
from _panda_moveit_commander import PandaMoveItCommander

class PandaCommander(ManipulationDriver):
  def __init__(self):

    self.move_group = rospy.get_param('~move_group', None)

    if not self.move_group:
      rospy.logerr('Unable to load move_group name from rosparam server path: move_group')
      sys.exit(1)

    rospy.wait_for_service('/franka_control/set_EE_frame')
    self.set_ee = rospy.ServiceProxy('/franka_control/set_EE_frame', SetEEFrame)

    rospy.wait_for_service('/franka_control/set_cartesian_impedance')
    self.cartesian_impedance_proxy = rospy.ServiceProxy('/franka_control/set_cartesian_impedance', FrankaSetCartesianImpedance)

    ManipulationDriver.__init__(self, PandaMoveItCommander(self.move_group))

    self.velocity_publisher = rospy.Publisher('/cartesian_velocity_node_controller/cartesian_velocity', Twist, queue_size=1)
    self.joint_velocity_publisher = rospy.Publisher('/joint_velocity_node_controller/joint_velocity', JointVelocity, queue_size=1)
    self.recover_on_estop = rospy.get_param('/manipulation_commander/recover_on_estop', True)

    # handling e-stop
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.state_cb)
    self.last_estop_state = 0

  def velocity_cb(self, msg):
    if self.switcher.get_current_name() != 'cartesian_velocity_node_controller':
        self.switcher.switch_controller('cartesian_velocity_node_controller')
    
    result = self.transform_velocity(msg, self.base_frame)

    self.velocity_publisher.publish(result)

  def joint_velocity_cb(self, msg):
    if self.switcher.get_current_name() != 'joint_velocity_node_controller':
        self.switcher.switch_controller('joint_velocity_node_controller')
    
    self.joint_velocity_publisher.publish(msg)

  def state_cb(self, msg):
    state = ManipulatorState()
    state.ee_pose = self.get_link_pose(self.base_frame, self.ee_frame) 
    
    state.cartesian_contact = msg.cartesian_contact
    state.cartesian_collision = msg.cartesian_collision
    
    state.errors |= ManipulatorState.ESTOP if msg.robot_mode == FrankaState.ROBOT_MODE_USER_STOPPED else 0
    state.errors |= ManipulatorState.COLLISION if any(state.cartesian_collision) else 0

    for n in msg.last_motion_errors.__slots__:
      if msg.robot_mode != 2 and getattr(msg.last_motion_errors, n):
        if n in ['joint_position_limits_violation', 
                 'joint_velocity_violation', 
                 'joint_position_motion_generator_start_pose_invalid',
                 'joint_motion_generator_position_limits_violation',
                 'joint_motion_generator_velocity_limits_violation',
                 'joint_motion_generator_velocity_discontinuity',
                 'joint_motion_generator_acceleration_discontinuity']:
          state.errors |= ManipulatorState.JOINT_LIMIT_VIOLATION   

        elif n in ['cartesian_position_limits_violation',
                   'cartesian_velocity_violation',
                   'cartesian_velocity_profile_safety_violation',
                   'cartesian_position_motion_generator_start_pose_invalid',
                   'cartesian_motion_generator_elbow_limit_violation',
                   'cartesian_motion_generator_velocity_limits_violation',
                   'cartesian_motion_generator_velocity_discontinuity',
                   'cartesian_motion_generator_acceleration_discontinuity',
                   'cartesian_motion_generator_elbow_sign_inconsistent',
                   'cartesian_motion_generator_start_elbow_invalid',
                   'cartesian_motion_generator_joint_position_limits_violation',
                   'cartesian_motion_generator_joint_velocity_limits_violation',
                   'cartesian_motion_generator_joint_velocity_discontinuity',
                   'cartesian_motion_generator_joint_acceleration_discontinuity',
                   'cartesian_position_motion_generator_invalid_frame']:
          state.errors |= ManipulatorState.CARTESIAN_LIMIT_VIOLATION

        elif n in ['force_control_safety_violation',
                   'joint_reflex',
                   'cartesian_reflex',
                   'force_controller_desired_force_tolerance_violation'
                   'joint_p2p_insufficient_torque_for_planning'
                   'tau_j_range_violation']:
          state.errors |= ManipulatorState.TORQUE_LIMIT_VIOLATION

        else:
          state.errors |= ManipulatorState.OTHER

    self.state_publisher.publish(state)

    if msg.robot_mode == FrankaState.ROBOT_MODE_IDLE:
      if self.recover_on_estop and self.last_estop_state == 1:
        self.moveit_commander.recover()
    else:
      if state.errors & ManipulatorState.OTHER == ManipulatorState.OTHER:
        self.moveit_commander.recover()
    
    self.last_estop_state = 1 if msg.robot_mode == FrankaState.ROBOT_MODE_USER_STOPPED else 0
    
  def recover_cb(self, req):
    self.moveit_commander.recover()
    return []

  def set_cartesian_impedance_cb(self, req):
    current = self.switcher.get_current_name()
    self.switcher.switch_controller(None)
    
    time.sleep(0.1)
    result = self.cartesian_impedance_proxy(req.cartesian_impedance)
    
    self.switcher.switch_controller(current)

    return SetCartesianImpedanceResponse(success=result.success, error=result.error)
    
  def set_ee_offset(self, offset):
    trans = transforms.pose_msg_to_trans(offset)
    return self.set_ee(SetEEFrameRequest(
      F_T_EE=list(trans.transpose().flatten())
    )).success
