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

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf/model.h>

#include "schunk_canopen_driver/SchunkCanopenHardwareInterface.h"

#include <icl_hardware_canopen/SchunkPowerBallNode.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

SchunkCanopenHardwareInterface::SchunkCanopenHardwareInterface (ros::NodeHandle& nh,
                                                                boost::shared_ptr<CanOpenController>& canopen)
  : m_node_handle (nh),
    m_canopen_controller (canopen)
{
  init();
}

void SchunkCanopenHardwareInterface::init()
{
  m_node_ids = m_canopen_controller->getNodeList();
  size_t num_nodes = m_node_ids.size();
  m_joint_position_commands.resize(num_nodes);
  m_joint_position_commands_last.resize(num_nodes);
  m_joint_positions.resize(num_nodes);
  m_joint_velocity.resize(num_nodes);
  m_joint_effort.resize(num_nodes);
  m_joint_names.clear();
  m_nodes_in_fault.resize(num_nodes, false);

  bool rosparam_limits_ok = true;
//  bool urdf_limits_ok = true;
//  bool urdf_soft_limits_ok = true;



//  boost::shared_ptr<urdf::ModelInterface> urdf = urdf::parseURDF("robot_description");

  urdf::Model urdf_model;
  if(!m_node_handle.hasParam("robot_description"))
  {
      ROS_ERROR("robot description not found!!");
  }
  if(!urdf_model.initParam("robot_description"))
  {
    ROS_ERROR("robot description parsing error!!");
  }



  // Initialize controller
  for (std::size_t i = 0; i < num_nodes; ++i) {
    std::string joint_name = "";
    std::string mapping_key = "~node_mapping_" + boost::lexical_cast<std::string>(static_cast<int>(m_node_ids[i]));
    ros::param::get(mapping_key, joint_name);
    m_joint_names.push_back(joint_name);
    ROS_DEBUG_STREAM("Controller Hardware interface: Loading joint with id " << static_cast<int>(m_node_ids[i]) << " named " << joint_name);
    if (joint_name == "")
    {
      ROS_ERROR_STREAM ("Could not find joint name for canopen device " << static_cast<int>(m_node_ids[i]) <<
        ". You will not be able to use this device with the controller!");
    }
    else
    {
      // Create joint state interface
      m_joint_state_interface.registerHandle(
          hardware_interface::JointStateHandle(m_joint_names[i],
              &m_joint_positions[i], &m_joint_velocity[i], &m_joint_effort[i]));

      // Create position joint interface
      hardware_interface::JointHandle hwi_handle(
          m_joint_state_interface.getHandle(m_joint_names[i]),
          &m_joint_position_commands[i]);
      m_position_joint_interface.registerHandle(hwi_handle);

      // Create a joint_limit_interface:

      // Populate (soft) joint limits from the ros parameter server
      rosparam_limits_ok = getJointLimits(joint_name, m_node_handle, m_joint_limits);
      if(!rosparam_limits_ok) {
        ROS_ERROR_STREAM ("Could not set the joint limits for joint " << joint_name << "!");
      }

      // Populate (soft) joint limits from URDF
      // Limits specified in URDF overwrite existing values in 'limits' and 'soft_limits'
      // Limits not specified in URDF preserve their existing values
/*      boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model.getJoint(joint_name);
      urdf_limits_ok = getJointLimits(urdf_joint, m_joint_limits);
      if(!urdf_limits_ok) {
        ROS_ERROR_STREAM ("Could not set the joint limits for joint " << joint_name << "!");
      }
      urdf_soft_limits_ok = getSoftJointLimits(urdf_joint, m_joint_soft_limits);
      if(!urdf_soft_limits_ok) {
        ROS_ERROR_STREAM ("Could not set the joint soft limits for joint " << joint_name << "!");
      }*/

      // Register handle in joint limits interface
      PositionJointSoftLimitsHandle limits_handle(hwi_handle, // We read the state and read/write the command
                                                  m_joint_limits,       // Limits spec
                                                  m_joint_soft_limits);  // Soft limits spec

      m_jnt_limits_interface.registerHandle(limits_handle);
    }
  }


  registerInterface(&m_joint_state_interface); // From RobotHW base class.
  registerInterface(&m_position_joint_interface); // From RobotHW base class.
}

void SchunkCanopenHardwareInterface::read()
{
  m_is_fault = false;
  m_joint_positions.resize(m_node_ids.size());
  SchunkPowerBallNode::Ptr node;
  for (size_t i = 0; i < m_node_ids.size(); ++i)
  {
    node = m_canopen_controller->getNode<SchunkPowerBallNode>(m_node_ids[i]);
    m_joint_positions[i] = node->getTargetFeedback();
    ds402::Statusword node_status = node->getStatus();
    m_is_fault |= node_status.bit.fault;
    if (node_status.bit.fault)
    {
      if (!m_nodes_in_fault[i])
      {
        ROS_ERROR_STREAM ("Node " << static_cast<int>(m_node_ids[i]) << " is in FAULT state");
      }
      m_nodes_in_fault[i] = true;
    }
    else
    {
      m_nodes_in_fault[i] = false;
    }
  }
}

void SchunkCanopenHardwareInterface::write(ros::Time time, ros::Duration period)
{
  if (m_node_ids.size() == m_joint_position_commands.size())
  {
    // For this position based interface, only the joint limits in degree and velocity are enforced. Acceleration is ignored.
    m_jnt_limits_interface.enforceLimits(period);

    std::stringstream commanded_positions;
    SchunkPowerBallNode::Ptr node;
    for (size_t i = 0; i < m_node_ids.size(); ++i)
    {
      const uint8_t& nr = m_node_ids[i];
      float pos = m_joint_position_commands[i];
      try
      {
        node = m_canopen_controller->getNode<SchunkPowerBallNode>(nr);
      }
      catch (const NotFoundException& e)
      {
        ROS_ERROR_STREAM ("One or more nodes could not be found in the controller. " << e.what());
        return;
      }
      m_canopen_controller->getNode<SchunkPowerBallNode>(nr)->setTarget(pos);
      commanded_positions << pos << " ";
    }
    if (m_joint_position_commands != m_joint_position_commands_last)
    {
      ROS_DEBUG_STREAM ("Commanded positions: " << commanded_positions.str());
      m_joint_position_commands_last = m_joint_position_commands;
    }
  }
  else
  {
    ROS_ERROR ("Number of known joints and number of commanded joints do not match!");
  }
}

bool SchunkCanopenHardwareInterface::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
    return hardware_interface::RobotHW::prepareSwitch(start_list, stop_list);
}

void SchunkCanopenHardwareInterface::doSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
    hardware_interface::RobotHW::doSwitch(start_list, stop_list);
}

sensor_msgs::JointState SchunkCanopenHardwareInterface::getJointMessage()
{
  sensor_msgs::JointState joint_msg;
  joint_msg.name = m_joint_names;
  joint_msg.header.stamp = ros::Time::now();
  joint_msg.position = m_joint_positions;
  joint_msg.velocity = m_joint_velocity;
  joint_msg.effort = m_joint_effort;

  return joint_msg;
}


