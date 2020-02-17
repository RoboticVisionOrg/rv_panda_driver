// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <rv_panda_driver/joint_velocity_node_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace rv_panda_driver {

bool JointVelocityNodeController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointVelocityNodeController: Could not get parameter arm_id");
    return false;
  }


  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }

  velocity_joint_interface_ =
      robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityNodeController: Could not get Joint velocity interface from "
        "hardware");
    return false;
  }
  
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityNodeController: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointVelocityNodeController: Exception getting state handle: " << e.what());
    return false;
  }


  node_handle.param<double>("max_duration_between_commands", max_duration_between_commands, 0.01);

  // Rate Limiting
  std::vector<double> limit;
  if(!node_handle.getParam("rate_limiting/velocity", limit)) {
    ROS_ERROR("JointVelocityNodeController: Could not get parameter rate_limiting/velocity");
    return false;
  }
  std::copy(limit.begin(), limit.end(), max_velocity.begin());

  if(!node_handle.getParam("rate_limiting/acceleration", limit)) {
    ROS_ERROR("JointVelocityNodeController: Could not get parameter rate_limiting/acceleration");
    return false;
  }
  std::copy(limit.begin(), limit.end(), max_acceleration.begin());

  if(!node_handle.getParam("rate_limiting/jerk", limit)) {
    ROS_ERROR("JointVelocityNodeController: Could not get parameter rate_limiting/jerk");
    return false;
  }
  std::copy(limit.begin(), limit.end(), max_jerk.begin());

  node_handle.param<bool>("stop_on_contact", stop_on_contact, true);

  velocity_command_subscriber = node_handle.subscribe("joint_velocity",
                                                       10,
                                                       &JointVelocityNodeController::joint_velocity_callback,
                                                       this);

  return true;
}

void JointVelocityNodeController::starting(const ros::Time& /* time */) {
  time_since_last_command = ros::Duration(0.0);
  velocity_command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  last_sent_velocity = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
}

void JointVelocityNodeController::joint_velocity_callback(const rv_msgs::JointVelocity::ConstPtr& msg) {
  if (msg->joints.size() != velocity_joint_handles_.size()) {
    ROS_ERROR("JointVelocityNodeController: Expects %d joint velocities, received %d joint velocities", velocity_joint_handles_.size(), msg->joints.size());
    return;
  }

  velocity_command[0] = msg->joints[0];
  velocity_command[1] = msg->joints[1];
  velocity_command[2] = msg->joints[2];
  velocity_command[3] = msg->joints[3];
  velocity_command[4] = msg->joints[4];
  velocity_command[5] = msg->joints[5];
  velocity_command[6] = msg->joints[6];

  time_since_last_command = ros::Duration(0.0);
}

void JointVelocityNodeController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  time_since_last_command += period;

  if(time_since_last_command.toSec() > max_duration_between_commands) {
    velocity_command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  }

  auto state = state_handle_->getRobotState();

  if(stop_on_contact) {
    for (size_t i = 0; i < state.cartesian_contact.size(); i++) {
      if(state.cartesian_contact[i]) {
        velocity_command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        // ROS_ERROR_STREAM("Detected Cartesian Contact in Direction "  << i);
      }
    }
  }

  last_sent_velocity = franka::limitRate(
    max_velocity,
    max_acceleration,
    max_jerk,
    velocity_command,
    state.dq_d,
    state.ddq_d
  );

  for (size_t idx = 0; idx < velocity_joint_handles_.size(); idx++) {
    velocity_joint_handles_[idx].setCommand(last_sent_velocity[idx]);
  }
}

void JointVelocityNodeController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace rv_panda_driver

PLUGINLIB_EXPORT_CLASS(rv_panda_driver::JointVelocityNodeController,
                       controller_interface::ControllerBase)
