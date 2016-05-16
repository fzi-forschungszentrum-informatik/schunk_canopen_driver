// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the SCHUNK Canopen Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// © Copyright 2016 SCHUNK GmbH, Lauffen/Neckar Germany
// © Copyright 2016 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch <mauch@fzi.de>
 * \date    2015-12-7
 *
 */
//----------------------------------------------------------------------

#ifndef SCHUNK_CANOPEN_HARDWARE_INTERFACE_H_
#define SCHUNK_CANOPEN_HARDWARE_INTERFACE_H_

#include <icl_hardware_canopen/CanOpenController.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>

using namespace icl_hardware;
using namespace canopen_schunk;

/*!
 * \brief This class defines a ros-control hardware interface.
 *
 */
class SchunkCanopenHardwareInterface : public hardware_interface::RobotHW
{
public:
  SchunkCanopenHardwareInterface (ros::NodeHandle& nh, boost::shared_ptr< CanOpenController >& canopen);

  /// \brief Initialize the hardware interface
  virtual void init();

  /// \brief Read the state from the robot hardware.
  virtual void read();

  /// \brief write the command to the robot hardware.
  virtual void write(ros::Time time, ros::Duration period);

  virtual bool prepareSwitch(
      const std::list<hardware_interface::ControllerInfo> &start_list,
      const std::list<hardware_interface::ControllerInfo> &stop_list);
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>&start_list,
      const std::list<hardware_interface::ControllerInfo>&stop_list);

  /*!
   * \brief Returns true, when at least one node in the hardware is in a fault state.
   */
  bool isFault() {return m_is_fault;}

  /*!
   * \brief Creates a joint_state message from the current joint angles and returns it.
   */
  sensor_msgs::JointState getJointMessage();

protected:
  ros::NodeHandle m_node_handle;
  boost::shared_ptr<CanOpenController> m_canopen_controller;

  // Interfaces
  hardware_interface::JointStateInterface m_joint_state_interface;
  hardware_interface::PositionJointInterface m_position_joint_interface;
  joint_limits_interface::PositionJointSoftLimitsInterface m_jnt_limits_interface;

  size_t m_num_joints;

  std::vector<uint8_t> m_node_ids;
  std::vector<std::string> m_joint_names;
  std::vector<double> m_joint_positions;
  std::vector<double> m_joint_velocity;
  std::vector<double> m_joint_effort;
  std::vector<double> m_joint_position_commands;
  std::vector<double> m_joint_position_commands_last;

  std::vector<bool> m_nodes_in_fault;
  bool m_is_fault;

  joint_limits_interface::JointLimits m_joint_limits;
  joint_limits_interface::SoftJointLimits m_joint_soft_limits; // only available through URDF, currently not used.
};



#endif /*SCHUNK_CANOPEN_HARDWARE_INTERFACE_H_*/
