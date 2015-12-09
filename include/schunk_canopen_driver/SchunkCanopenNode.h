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
#include "std_srvs/Trigger.h"
#include "schunk_canopen_driver/HomeAll.h"
#include "schunk_canopen_driver/HomeWithIDs.h"
#include "schunk_canopen_driver/HomeWithJointNames.h"


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

  // Action interfaces for standalone mode (without ros_control)
  void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);
  void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);
  void trajThread(actionlib::ServerGoalHandle< control_msgs::FollowJointTrajectoryAction >& gh);

  // Service callback handlers
  bool enableNodes(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);
  bool quickStopNodes(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);
  bool homeNodesCanIds(schunk_canopen_driver::HomeWithIDsRequest& req,
                       schunk_canopen_driver::HomeWithIDsResponse& resp);
  bool homeNodesJointNames(schunk_canopen_driver::HomeWithJointNamesRequest& req,
                           schunk_canopen_driver::HomeWithJointNamesResponse& resp);
  bool homeAllNodes(schunk_canopen_driver::HomeAllRequest& req,
                    schunk_canopen_driver::HomeAllResponse& resp);
  bool closeBrakes (std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);

  void rosControlLoop ();

  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> m_action_server;
  ros::ServiceServer m_enable_service;
  ros::ServiceServer m_close_brakes_service;
  ros::ServiceServer m_quick_stop_service;
  ros::ServiceServer m_home_service_all;
  ros::ServiceServer m_home_service_joint_names;
  ros::ServiceServer m_home_service_canopen_ids;

  ros::Publisher m_joint_pub;


  CanOpenController::Ptr m_controller;

  std::vector<DS402Group::Ptr> m_chain_handles;
  std::map<std::string, uint8_t> m_joint_name_mapping;
  bool m_has_goal;
  boost::thread m_traj_thread;
  boost::thread m_ros_control_thread;

  bool m_use_ros_control;

  boost::shared_ptr<SchunkCanopenHardwareInterface> m_hardware_interface;
  boost::shared_ptr<controller_manager::ControllerManager> m_controller_manager;

  bool m_was_disabled;
  bool m_is_enabled;
  bool m_homing_active;

};

#endif /* SCHUNK_CANOPEN_NODE_H_ */
