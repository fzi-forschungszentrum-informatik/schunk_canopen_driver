// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch <mauch@fzi.de>
 * \date    2015-12-3
 *
 */
//----------------------------------------------------------------------



#ifndef SCHUNK_CANOPEN_NODE_H_
#define SCHUNK_CANOPEN_NODE_H_

#include <ros/ros.h>

#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/action_server.h"
#include "actionlib/server/server_goal_handle.h"

#include "SchunkCanopenHardwareInterface.h"

#include <icl_hardware_canopen/CanOpenController.h>

using namespace icl_hardware;
using namespace canopen_schunk;

class SchunkCanopenNode
{
public:
  SchunkCanopenNode();


private:
  ros::NodeHandle m_priv_nh;
  ros::NodeHandle m_pub_nh;

  void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);
  void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> m_action_server;
  void trajThread(actionlib::ServerGoalHandle< control_msgs::FollowJointTrajectoryAction >& gh);

  void rosControlLoop ();



  CanOpenController::Ptr m_controller;

  std::vector<DS402Group::Ptr> m_chain_handles;
  bool m_has_goal;
  boost::thread m_traj_thread;
  boost::thread m_ros_control_thread;

  bool m_use_ros_control;

  boost::shared_ptr<SchunkCanopenHardwareInterface> m_hardware_interface;
  boost::shared_ptr<controller_manager::ControllerManager> m_controller_manager;

};

#endif /* SCHUNK_CANOPEN_NODE_H_ */
