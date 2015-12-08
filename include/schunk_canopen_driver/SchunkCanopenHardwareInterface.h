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

#ifndef SCHUNK_CANOPEN_HARDWARE_INTERFACE_H_
#define SCHUNK_CANOPEN_HARDWARE_INTERFACE_H_

#include <icl_hardware_canopen/CanOpenController.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

using namespace icl_hardware;
using namespace canopen_schunk;

class SchunkCanopenHardwareInterface : public hardware_interface::RobotHW
{
public:
  SchunkCanopenHardwareInterface (ros::NodeHandle& nh, boost::shared_ptr< CanOpenController >& canopen);

  /// \brief Initialize the hardware interface
  virtual void init();

  /// \brief Read the state from the robot hardware.
  virtual void read();

  /// \brief write the command to the robot hardware.
  virtual void write();

  bool canSwitch(
      const std::list<hardware_interface::ControllerInfo> &start_list,
      const std::list<hardware_interface::ControllerInfo> &stop_list) const;
  void doSwitch(const std::list<hardware_interface::ControllerInfo>&start_list,
      const std::list<hardware_interface::ControllerInfo>&stop_list);

  bool isFault() {return m_is_fault;}

protected:
  ros::NodeHandle m_node_handle;
  boost::shared_ptr<CanOpenController> m_canopen_controller;

  // Interfaces
  hardware_interface::JointStateInterface m_joint_state_interface;
  hardware_interface::PositionJointInterface m_position_joint_interface;

  size_t m_num_joints;

  std::vector<uint8_t> m_node_ids;
  std::vector<std::string> m_joint_names;
  std::vector<double> m_joint_positions;
  std::vector<double> m_joint_velocity;
  std::vector<double> m_joint_effort;
  std::vector<double> m_joint_position_commands;
  std::vector<double> m_joint_position_commands_last;

  bool m_is_fault;
};



#endif /*SCHUNK_CANOPEN_HARDWARE_INTERFACE_H_*/
