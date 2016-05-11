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


#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
#include "ros/service_server.h"
#include "std_msgs/Int16MultiArray.h"


#include "schunk_canopen_driver/SchunkCanopenNode.h"
#include <icl_hardware_canopen/SchunkPowerBallNode.h>
#include "icl_core_logging/Logging.h"




SchunkCanopenNode::SchunkCanopenNode()
  : m_priv_nh("~"),
    m_action_server(m_priv_nh, "follow_joint_trajectory",
    boost::bind(&SchunkCanopenNode::goalCB, this, _1),
    boost::bind(&SchunkCanopenNode::cancelCB, this, _1), false),
    m_has_goal(false),
    m_use_ros_control(false),
    m_was_disabled(false),
    m_is_enabled(false),
    m_homing_active(false),
    m_nodes_initialized(false)
{
  std::string can_device_name;
  int frequency = 30;
  std::vector<std::string> chain_names;
  std::map<std::string, std::vector<int> > chain_configuratuions;
  m_chain_handles.clear();
  sensor_msgs::JointState joint_msg;
  bool autostart = true;

  m_priv_nh.getParam("chain_names", chain_names);
  ros::param::get("~use_ros_control", m_use_ros_control);
  m_priv_nh.param<std::string>("can_device_name", can_device_name, "auto");
  m_priv_nh.param<float>  ("ppm_profile_velocity",       m_ppm_config.profile_velocity,         0.2);
  m_priv_nh.param<float>  ("ppm_profile_acceleration",   m_ppm_config.profile_acceleration,     0.2);
  m_priv_nh.param<bool>   ("ppm_use_relative_targets",   m_ppm_config.use_relative_targets,     false);
  m_priv_nh.param<bool>   ("ppm_change_set_immediately", m_ppm_config.change_set_immediately,   true);
  m_priv_nh.param<bool>   ("ppm_use_blending",           m_ppm_config.use_blending,             true);
  m_priv_nh.param<bool>   ("autostart",                  autostart,                             true);
  m_priv_nh.param<std::string>("traj_controller_name", m_traj_controller_name, "pos_based_pos_traj_controller");

  // Create a canopen controller
  try
  {
    m_controller = boost::make_shared<CanOpenController>(can_device_name);
  }
  catch (const DeviceException& e)
  {
    ROS_ERROR_STREAM ("Initializing CAN device failed. Reason: " << e.what());
    ROS_INFO ("Shutting down now.");
    return;
  }

  // Load SCHUNK powerball specific error codes
  char const* tmp = std::getenv("CANOPEN_RESOURCE_PATH");
  if (tmp == NULL)
  {
    LOGGING_WARNING_C(
        CanOpen,
        CanOpenController,
        "The environment variable 'CANOPEN_RESOURCE_PATH' could not be read. No Schunk specific error codes will be used." << endl);
  }
  else
  {
    std::string emcy_emergency_errors_filename = boost::filesystem::path(tmp / boost::filesystem::path("EMCY_schunk.ini")).string();
    EMCY::addEmergencyErrorMap( emcy_emergency_errors_filename, "schunk_error_codes");
  }


  // Get chain configuration from parameter server
  ROS_INFO_STREAM ("Can device identifier: " << can_device_name);
  ROS_INFO_STREAM ("Found " << chain_names.size() << " chains");

  for (size_t i = 0; i < chain_names.size(); ++i)
  {
    std::string name = "chain_" + chain_names[i];
    m_controller->addGroup<DS402Group>(chain_names[i]);
    m_chain_handles.push_back(m_controller->getGroup<DS402Group>(chain_names[i]));
    std::vector<int> chain;
    try
    {
      m_priv_nh.getParam(name, chain);
    }
    catch (ros::InvalidNameException e)
    {
      ROS_ERROR_STREAM("Parameter Error!");
    }
    if (chain.size() == 0)
    {
      ROS_ERROR_STREAM("Did not find device list for chain " << chain_names[i] << ". Make sure, that an entry " << name << " exists.");
      continue;
    }

    // Get the chain type
    name = "chain_" + chain_names[i] + "_type";
    std::string chain_type = "PowerBall";
    try
    {
      m_priv_nh.getParam(name, chain_type);
    }
    catch (ros::InvalidNameException e)
    {
      ROS_ERROR_STREAM("Parameter Error!");
    }

    // Get the chain transmission factor
    name = "chain_" + chain_names[i] + "_transmission_factor";
    double transmission_factor = 1.0;
    try
    {
      m_priv_nh.getParam(name, transmission_factor);
      if (chain_type != "PowerBall")
      {
        ROS_INFO_STREAM ("Chain transmission factor is " << transmission_factor);
      }
    }
    catch (ros::InvalidNameException e)
    {
      ROS_ERROR_STREAM("Parameter Error!");
    }


    ROS_INFO_STREAM ("Found chain with name " << chain_names[i] << " of type " << chain_type << " containing " << chain.size() << " nodes.");
    chain_configuratuions[name] = chain;
    for (size_t j = 0; j < chain.size(); ++j)
    {
      if (chain_type == "DS402")
      {
        m_controller->addNode<DS402Node>(chain[j], chain_names[i]);
        m_controller->getNode<DS402Node>(chain[j])->setTransmissionFactor(transmission_factor);
      }
      else
      {
        m_controller->addNode<SchunkPowerBallNode>(chain[j], chain_names[i]);
      }

      std::string joint_name = "";
      std::string mapping_key = "~node_mapping_" + boost::lexical_cast<std::string>( chain[j]);
      ros::param::get(mapping_key, joint_name);

      m_joint_name_mapping[joint_name] =  static_cast<uint8_t>(chain[j]);
      joint_msg.name.push_back(joint_name);
    }
  }

  if (autostart)
  {
    initDevices();
  }
  else
  {
    m_init_service = m_priv_nh.advertiseService("init_devices",
     &SchunkCanopenNode::initDevicesCb, this);
  }


  ros::Rate loop_rate(frequency);

  DS402Node::Ptr node;
  std_msgs::Int16MultiArray currents;

  // The robot's status (joint values and currents) will be published periodically in here
  while (ros::ok())
  {
    ros::spinOnce();

    if (m_nodes_initialized)
    {
      joint_msg.position.clear();
      currents.data.clear();
      // TODO: Go over all groups. to handle different things. For example, we might not want to
      // read the current from a gripper...
      for (std::map<std::string, uint8_t>::iterator it = m_joint_name_mapping.begin();
           it != m_joint_name_mapping.end();
           ++it)
      {
        const uint8_t& nr = it->second;
        node = m_controller->getNode<DS402Node>(nr);
        joint_msg.position.push_back(node->getTargetFeedback());

        // Schunk nodes write currents into the torque_actual register
        try
        {
          currents.data.push_back(node->getTPDOValue<int16_t>("measured_torque"));
        }
        catch (PDOException& e)
        {
          ROS_ERROR_STREAM(e.what());
          currents.data.push_back(0);
        }
      }
      joint_msg.header.stamp = ros::Time::now();
      m_joint_pub.publish(joint_msg);

      m_current_pub.publish(currents);
    }
    loop_rate.sleep();
  }
}


bool SchunkCanopenNode::initDevicesCb(std_srvs::TriggerRequest& req,
                                      std_srvs::TriggerResponse& resp)
{
  initDevices();
  resp.success = true;
  return resp.success;
}


void SchunkCanopenNode::initDevices()
{

  // initialize all nodes, by default this will start ProfilePosition mode, so we're good to enable nodes
  try {
    m_controller->initNodes();
  }
  catch (const ProtocolException& e)
  {
    ROS_ERROR_STREAM ("Caught ProtocolException while initializing devices: " << e.what());
    ROS_INFO ("Going to shut down now");
    exit (-1);
  }
  catch (const PDOException& e)
  {
    ROS_ERROR_STREAM ("Caught PDOException while initializing devices: " << e.what());
    ROS_INFO ("Going to shut down now");
    exit (-1);
  }

  ds402::eModeOfOperation mode = ds402::MOO_PROFILE_POSITION_MODE;
  if (m_use_ros_control)
  {
    mode = ds402::MOO_INTERPOLATED_POSITION_MODE;
    m_hardware_interface.reset(
      new SchunkCanopenHardwareInterface(m_pub_nh, m_controller));
    m_controller_manager.reset(
      new controller_manager::ControllerManager( m_hardware_interface.get(), m_pub_nh));

  }

  // Start interface (either action server or ros_control)

  if (m_use_ros_control)
  {
    m_ros_control_thread = boost::thread(&SchunkCanopenNode::rosControlLoop, this);
  }
  else
  {
    for (size_t i = 0; i < m_chain_handles.size(); ++i)
    {
      try {
        m_chain_handles[i]->setupProfilePositionMode(m_ppm_config);
        m_chain_handles[i]->enableNodes(mode);
      }
      catch (const ProtocolException& e)
      {
        ROS_ERROR_STREAM ("Caught ProtocolException while enabling nodes from chain " <<
          m_chain_handles[i]->getName() << ". Nodes from this group won't be enabled.");
        continue;
      }
      ROS_INFO_STREAM ("Enabled nodes from chain " << m_chain_handles[i]->getName());
    }
    m_action_server.start();
  }

  // Create joint state publisher
  m_joint_pub = m_pub_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  m_current_pub = m_pub_nh.advertise<std_msgs::Int16MultiArray>("joint_currents", 1);

  // services
  m_enable_service =  m_priv_nh.advertiseService("enable_nodes", &SchunkCanopenNode::enableNodes, this);
  m_close_brakes_service =  m_priv_nh.advertiseService("close_brakes",
     &SchunkCanopenNode::closeBrakes, this);
  m_quick_stop_service =  m_priv_nh.advertiseService("quick_stop_nodes",
     &SchunkCanopenNode::quickStopNodes, this);
  m_home_service_all = m_priv_nh.advertiseService("home_reset_offset_all",
     &SchunkCanopenNode::homeAllNodes, this);
  m_home_service_joint_names = m_priv_nh.advertiseService("home_reset_offset_by_id",
     &SchunkCanopenNode::homeNodesCanIds, this);
  m_home_service_canopen_ids = m_priv_nh.advertiseService("home_reset_offset_by_name",
     &SchunkCanopenNode::homeNodesJointNames, this);

  m_nodes_initialized = true;
}


void SchunkCanopenNode::goalCB (actionlib::ServerGoalHandle< control_msgs::FollowJointTrajectoryAction > gh)
{
  ROS_INFO ("Executing Trajectory action");

  /* TODO: Catch errors:
   * - Joint not enabled
   * - EmergencyStopState
   * - Overwriting trajectory
   * - invalid positions
   */


  if (gh.getGoal()->trajectory.joint_names.size() != gh.getGoal()->trajectory.points[0].positions.size())
  {
    ROS_ERROR_STREAM ("Number of given joint names (" << gh.getGoal()->trajectory.joint_names.size() <<
      ") and joint states (" << gh.getGoal()->trajectory.points.size() << ") do not match! Aborting goal!");
    return;
  }

  if (m_has_goal)
  {
    ROS_WARN_STREAM ("Sent a new goal while another goal was still running. Depending on the " <<
      "device configuration this will either result in a queuing or the current trajectory " <<
      "will be overwritten."
    );
  }


  gh.setAccepted();
  m_has_goal = true;
  m_traj_thread = boost::thread(&SchunkCanopenNode::trajThread, this, gh);

}

void SchunkCanopenNode::trajThread(actionlib::ServerGoalHandle< control_msgs::FollowJointTrajectoryAction >& gh)
{
  control_msgs::FollowJointTrajectoryActionResult result;
  control_msgs::FollowJointTrajectoryActionFeedback feedback;
  feedback.feedback.header = gh.getGoal()->trajectory.header;
  result.header = gh.getGoal()->trajectory.header;

  ros::Time start = ros::Time::now();
  bool targets_reached = true;

  for (size_t waypoint = 0; waypoint < gh.getGoal()->trajectory.points.size(); ++waypoint)
  {
    feedback.feedback.desired.positions.clear();
    feedback.feedback.joint_names.clear();
    feedback.feedback.actual.positions.clear();
    for (size_t i = 0; i < gh.getGoal()->trajectory.joint_names.size(); ++i)
    {

      uint8_t nr = m_joint_name_mapping[gh.getGoal()->trajectory.joint_names[i]];
      float pos = boost::lexical_cast<float>(gh.getGoal()->trajectory.points[waypoint].positions[i]);
      ROS_INFO_STREAM ("Joint " << static_cast<int>(nr) << ": " << pos);
      SchunkPowerBallNode::Ptr node;
      try
      {
        node = m_controller->getNode<SchunkPowerBallNode>(nr);
      }
      catch (const NotFoundException& e)
      {
        ROS_ERROR_STREAM ("One or more nodes could not be found in the controller. " << e.what());
        result.result.error_code = -2;
        result.result.error_string = e.what();
        gh.setAborted(result.result);
        return;
      }
      m_controller->getNode<SchunkPowerBallNode>(nr)->setTarget(pos);
      feedback.feedback.desired.positions.push_back(pos);
      feedback.feedback.joint_names.push_back(gh.getGoal()->trajectory.joint_names[i]);


      pos = node->getTargetFeedback();
      feedback.feedback.actual.positions.push_back(pos);
    }

    m_controller->syncAll();
    m_controller->enablePPMotion();

    ros::Duration max_time = gh.getGoal()->goal_time_tolerance;

    ros::Duration spent_time = start - start;
    std::vector<bool> reached_vec;

    // Give the brakes time to open up
    sleep(1);

    while ( spent_time < max_time || max_time.isZero() )
    {
      try {
        m_controller->syncAll();
      }
      catch (const std::exception& e)
      {
        ROS_ERROR_STREAM (e.what());
        gh.setAborted();
        return;
      }
      usleep(10000);

      bool waypoint_reached = true;
      for (size_t i = 0; i < gh.getGoal()->trajectory.joint_names.size(); ++i)
      {
        uint8_t nr = m_joint_name_mapping[gh.getGoal()->trajectory.joint_names[i]];
        SchunkPowerBallNode::Ptr node = m_controller->getNode<SchunkPowerBallNode>(nr);
        waypoint_reached &= node->isTargetReached();
        float pos = node->getTargetFeedback();
  //       ROS_INFO_STREAM ("Node " << nr << " target reached: " << waypoint_reached <<
  //         ", position is: " << pos
  //       );
        feedback.feedback.actual.time_from_start = spent_time;
        feedback.feedback.actual.positions.at(i) = (pos);
      }


      gh.publishFeedback(feedback.feedback);
      targets_reached = waypoint_reached;



      if (waypoint_reached)
      {
        ROS_INFO_STREAM ("Waypoint " << waypoint <<" reached" );
        break;
      }
      spent_time = ros::Time::now() - start;
      if (spent_time > max_time && !max_time.isZero())
      {
        result.result.error_code = -5;
        result.result.error_string = "Did not reach targets in specified time";
        gh.setAborted();
        m_has_goal = false;
        return;
      }
      if (boost::this_thread::interruption_requested() )
      {
        ROS_ERROR ("Interruption requested");
        m_has_goal = false;
        return;
      }
    }
  }

  if (targets_reached)
  {
    ROS_INFO ("All targets reached" );
    result.result.error_code = 0;
    result.result.error_string = "All targets successfully reached";
    gh.setSucceeded(result.result);
  }
  else
  {
    ROS_INFO ("Not all targets reached" );
    result.result.error_code = -5;
    result.result.error_string = "Did not reach targets in specified time";
    gh.setAborted();
  }
  m_has_goal = false;
}


void SchunkCanopenNode::cancelCB (actionlib::ServerGoalHandle< control_msgs::FollowJointTrajectoryAction > gh)
{
  m_is_enabled = false;
  ROS_INFO ("Canceling Trajectory action");

  m_traj_thread.interrupt();
  ROS_INFO ("Stopped trajectory thread");

  for (size_t i = 0; i < m_chain_handles.size(); ++i)
  {
    m_chain_handles[i]->quickStop();
  }
  ROS_INFO ("Device stopped");
  sleep(1);
  for (size_t i = 0; i < m_chain_handles.size(); ++i)
  {
    m_chain_handles[i]->enableNodes();
    m_is_enabled = true;
  }

  control_msgs::FollowJointTrajectoryActionResult result;
  result.result.error_code = 0;
  result.result.error_string = "Trajectory preempted by user";

  gh.setCanceled(result.result);
}

void SchunkCanopenNode::rosControlLoop()
{
  ros::Duration elapsed_time;
  ros::Time last_time = ros::Time::now();
  ros::Time current_time = ros::Time::now();

  m_controller->syncAll();
  sleep(0.5);

  for (size_t i = 0; i < m_chain_handles.size(); ++i)
  {
    try {
      m_chain_handles[i]->enableNodes(ds402::MOO_INTERPOLATED_POSITION_MODE);
    }
    catch (const ProtocolException& e)
    {
      ROS_ERROR_STREAM ("Caught ProtocolException while enabling nodes");
      continue;
    }
    m_controller->syncAll();
  }
  m_is_enabled = true;

  size_t counter = 0;

  while (ros::ok() && !m_homing_active) {
    current_time = ros::Time::now();
    elapsed_time = current_time - last_time;
    last_time = current_time;
    // Input
    m_hardware_interface->read();
    sensor_msgs::JointState joint_msg = m_hardware_interface->getJointMessage();
    if (m_joint_pub)
    {
      m_joint_pub.publish (joint_msg);
    }
    if (m_hardware_interface->isFault() && m_is_enabled)
    {
      m_controller_manager->getControllerByName(m_traj_controller_name)->stopRequest(ros::Time::now());
      ROS_ERROR ("Some nodes are in FAULT state! No commands will be sent. Once the fault is removed, call the enable_nodes service.");
      m_is_enabled = false;
    }
    // Control
    m_controller->syncAll();
    if (m_was_disabled && m_is_enabled)
    {
      ROS_INFO ("Recovering from FAULT state. Resetting controller");
      m_controller_manager->update(current_time, elapsed_time, true);
      m_was_disabled = false;
    }
    else if (m_is_enabled)
    {
      m_controller_manager->update(current_time, elapsed_time);
      /* Give the controller some time, otherwise it will send a 0 waypoint vector which
       * might lead into a following error, if joints are not at 0
       * TODO: Find a better solution for that.
       */
      if (counter > 20)
      {
      // Output
        m_hardware_interface->write(current_time, elapsed_time);
      }
    }

//     node->printStatus();


    ++counter;
    usleep(10000);
  }

  ROS_INFO ("Shutting down ros_control_loop thread.");
}

bool SchunkCanopenNode::enableNodes(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp)
{
  m_controller_manager->getControllerByName(m_traj_controller_name)->startRequest(ros::Time::now());
  try
  {
    for (size_t i = 0; i < m_chain_handles.size(); ++i)
    {
      m_chain_handles[i]->enableNodes();
    }
  }
  catch (const ProtocolException& e)
  {
    ROS_ERROR_STREAM ( "Error while enabling nodes: " << e.what());
  }
  m_was_disabled = true;
  m_is_enabled = true;
  resp.success = true;
  return true;
}

bool SchunkCanopenNode::closeBrakes(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp)
{
  try
  {
    for (size_t i = 0; i < m_chain_handles.size(); ++i)
    {
      m_chain_handles[i]->closeBrakes();
    }
  }
  catch (const ProtocolException& e)
  {
    ROS_ERROR_STREAM ( "Error while enabling nodes: " << e.what());
  }
  resp.success = true;
  m_was_disabled = true;
  m_is_enabled = false;
  m_controller_manager->getControllerByName(m_traj_controller_name)->stopRequest(ros::Time::now());

  ROS_INFO ("Closed brakes for all nodes. For reenabling, please use the enable_nodes service. Thank you for your attention.");
  return true;
}

bool SchunkCanopenNode::quickStopNodes(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp)
{
  m_is_enabled = false;
  if (!m_use_ros_control)
  {
    m_traj_thread.interrupt();
    ROS_INFO ("Stopped trajectory thread");
  }

  try
  {
    for (size_t i = 0; i < m_chain_handles.size(); ++i)
    {
      m_chain_handles[i]->quickStop();
    }
  }
  catch (const ProtocolException& e)
  {
    ROS_ERROR_STREAM ( "Error while quick stopping nodes: " << e.what());
  }
  resp.success = true;
  m_was_disabled = false;
  m_controller_manager->getControllerByName(m_traj_controller_name)->stopRequest(ros::Time::now());
  ROS_INFO ("Quick stopped all nodes. For reenabling, please use the enable_nodes service. Thank you for your attention.");
  return true;
}

bool SchunkCanopenNode::homeAllNodes(schunk_canopen_driver::HomeAllRequest& req,
                                     schunk_canopen_driver::HomeAllResponse& resp)
{
  schunk_canopen_driver::HomeWithIDsRequest req_fwd;
  schunk_canopen_driver::HomeWithIDsResponse resp_fwd;
  req_fwd.node_ids = m_controller->getNodeList();


  homeNodesCanIds (req_fwd, resp_fwd);
  resp.success = resp_fwd.success;
  return resp.success;
}


bool SchunkCanopenNode::homeNodesJointNames(schunk_canopen_driver::HomeWithJointNamesRequest& req,
                                            schunk_canopen_driver::HomeWithJointNamesResponse& resp)
{
  schunk_canopen_driver::HomeWithIDsRequest req_fwd;
  schunk_canopen_driver::HomeWithIDsResponse resp_fwd;
  for (std::vector<std::string>::iterator it = req.joint_names.begin();
       it != req.joint_names.end();
  ++it)
  {
    if (m_joint_name_mapping.find(*it) != m_joint_name_mapping.end())
    {
      req_fwd.node_ids.push_back(m_joint_name_mapping[*it]);
    }
    else
    {
      ROS_ERROR_STREAM ("Could not find joint " << *it << ". No homing will be performed for this joint.");
    }
  }
  homeNodesCanIds (req_fwd, resp_fwd);
  resp.success = resp_fwd.success;
  return resp.success;
}


bool SchunkCanopenNode::homeNodesCanIds(schunk_canopen_driver::HomeWithIDsRequest& req,
                                        schunk_canopen_driver::HomeWithIDsResponse& resp)
{
  try
  {
    for (size_t i = 0; i < m_chain_handles.size(); ++i)
    {
      m_chain_handles[i]->disableNodes();
    }
  }
  catch (const ProtocolException& e)
  {
    ROS_ERROR_STREAM ( "Error while stopping nodes: " << e.what());
  }

  m_homing_active = true;
  m_is_enabled = false;

  for (std::vector<uint8_t>::iterator it = req.node_ids.begin(); it != req.node_ids.end(); ++it)
  {
    const uint8_t& id = *it;
    SchunkPowerBallNode::Ptr node = m_controller->getNode<SchunkPowerBallNode>(id);
    try
    {
      node->home();
    }
    catch (const DeviceException& e)
    {
      ROS_ERROR_STREAM ( "Error while homing node " << static_cast<int>(id) << ": " << e.what());
    }
  }

  if (m_use_ros_control)
  {
    m_ros_control_thread = boost::thread(&SchunkCanopenNode::rosControlLoop, this);
  }

  m_homing_active = false;
  m_was_disabled = true;
  m_is_enabled = true;
  resp.success = true;
  return resp.success;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "schunk_chain");
  icl_core::logging::initialize(argc, argv);

  SchunkCanopenNode my_schunk_node;

  return 0;
}
