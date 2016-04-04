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
  /*!
   * \brief Control loop thread when not using ros_control
   */
  void trajThread(actionlib::ServerGoalHandle< control_msgs::FollowJointTrajectoryAction >& gh);


  /*!
   * \brief This service call enables the devices after a fault, a quick stop or simply
   * when the brakes have been closed by hand (via the close_brakes service call)
   */
  bool enableNodes(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);

  /*!
   * \brief Performs a quick stop on all nodes. You should prefer this service call
   * to aborting the followJointTrajectory action, as this will instantaniously stop
   * the robot's movement without resulting in a fault state.
   */
  bool quickStopNodes(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);

  /*!
   * \brief Perform a reset offset for a given list of nodes. You should call this
   * service, after driving the device to it's zero position manually. Only the nodes
   * given in the node list are affected by this service call. The position of all unlisted
   * joints is irrelevant.
   *
   * \note: This service call exists for canopen IDs (usually 3-8 for a lwa4p) and
   * as an interface using the joint names from the URDF.
   */
  bool homeNodesCanIds(schunk_canopen_driver::HomeWithIDsRequest& req,
                       schunk_canopen_driver::HomeWithIDsResponse& resp);

  /*!
   * \brief Perform a reset offset for a given list of nodes. You should call this
   * service, after driving the device to it's zero position manually. Only the nodes
   * given in the node list are affected by this service call. The position of all unlisted
   * joints is irrelevant.
   *
   * \note: This service call exists for canopen IDs (usually 3-8 for a lwa4p) and
   * as an interface using the joint names from the URDF.
   */
  bool homeNodesJointNames(schunk_canopen_driver::HomeWithJointNamesRequest& req,
                           schunk_canopen_driver::HomeWithJointNamesResponse& resp);

  /*!
   * \brief Perform a reset offset for all nodes. You should call this
   * service, after driving the devices to their zero position manually.
   */
  bool homeAllNodes(schunk_canopen_driver::HomeAllRequest& req,
                    schunk_canopen_driver::HomeAllResponse& resp);


  /*!
   * \brief Close the brakes on all nodes. Use this when the device is standing still
   * for a longer time to reduce hight motor currents for doing nothing.
   *
   * Remember to call the enable_nodes service when you want to go on.
   */
  bool closeBrakes (std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);

  /*!
   * \brief Triggers initialization of the canopen devices.
   *
   * This service will only be advertised when the autostart parameter is set to
   * false. Otherwise initialization will be triggered automatically at startup.
   */
  bool initDevicesCb (std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);

  /*!
   * \brief Function that actually initializes the devices. Will get called either on
   * startup (if autostart is allowed) or by the init_devices service call.
   */
  void initDevices();

  /*!
   * \brief When using ros_control this will perform the control loop in a separate thread.
   */
  void rosControlLoop ();


  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> m_action_server;
  ros::ServiceServer m_enable_service;
  ros::ServiceServer m_close_brakes_service;
  ros::ServiceServer m_quick_stop_service;
  ros::ServiceServer m_home_service_all;
  ros::ServiceServer m_home_service_joint_names;
  ros::ServiceServer m_home_service_canopen_ids;
  ros::ServiceServer m_init_service;

  ros::Publisher m_joint_pub;
  ros::Publisher m_current_pub;


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

  ds402::ProfilePositionModeConfiguration m_ppm_config;
  bool m_nodes_initialized;

  std::string m_traj_controller_name;

};

#endif /* SCHUNK_CANOPEN_NODE_H_ */
