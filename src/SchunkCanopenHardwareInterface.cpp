// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch <mauch@fzi.de>
 * \date    2015-12-7
 *
 */
//----------------------------------------------------------------------


#include "schunk_canopen_driver/SchunkCanopenHardwareInterface.h"

#include <icl_hardware_canopen/SchunkPowerBallNode.h>

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
  m_joint_position_commands.resize(m_node_ids.size());
  m_joint_position_commands_last.resize(m_node_ids.size());
  m_joint_positions.resize(m_node_ids.size());
  m_joint_velocity.resize(m_node_ids.size());
  m_joint_effort.resize(m_node_ids.size());
  m_joint_names.clear();

  // Initialize controller
  for (std::size_t i = 0; i < m_node_ids.size(); ++i) {
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
      m_position_joint_interface.registerHandle(
          hardware_interface::JointHandle(
              m_joint_state_interface.getHandle(m_joint_names[i]),
              &m_joint_position_commands[i]));
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
      ROS_ERROR_STREAM ("Node " << static_cast<int>(m_node_ids[i]) << " is in FAULT state");
    }
  }
}

void SchunkCanopenHardwareInterface::write()
{
  if (m_node_ids.size() == m_joint_position_commands.size())
  {
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
      ROS_INFO_STREAM ("Commanded positions: " << commanded_positions.str());
      m_joint_position_commands_last = m_joint_position_commands;
    }
  }
  else
  {
    ROS_ERROR ("Number of known joints and number of commanded joints do not match!");
  }
}

bool SchunkCanopenHardwareInterface::canSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list) const
{
    return hardware_interface::RobotHW::canSwitch(start_list, stop_list);
}

void SchunkCanopenHardwareInterface::doSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
    hardware_interface::RobotHW::doSwitch(start_list, stop_list);
}


