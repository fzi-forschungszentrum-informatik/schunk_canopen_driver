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


#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
#include "ros/service_server.h"


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
    m_was_disabled(true),
    m_is_enabled(false),
    m_homing_active(false)
{
  std::string can_device_name;
  uint8_t first_node;
  int frequency = 30;
  std::vector<std::string> chain_names;
  std::map<std::string, std::vector<int> > chain_configuratuions;
  m_chain_handles.clear();
  sensor_msgs::JointState joint_msg;

  ds402::ProfilePositionModeConfiguration config;
  config.profile_acceleration = 0.2;
  config.profile_velocity = 0.2;
  config.use_relative_targets = false;
  config.change_set_immediately = true;
  config.use_blending = true;

  m_priv_nh.param<std::string>("can_device_name", can_device_name, "/dev/pcanusb1");
  m_priv_nh.getParam("chain_names", chain_names);
  ros::param::get("~use_ros_control", m_use_ros_control);
  m_priv_nh.getParam("ppm_profile_velocity", config.profile_velocity);
  m_priv_nh.getParam("ppm_profile_acceleration", config.profile_acceleration);
  m_priv_nh.getParam("ppm_use_relative_targets", config.use_relative_targets);
  m_priv_nh.getParam("ppm_change_set_immediately", config.change_set_immediately);
  m_priv_nh.getParam("ppm_use_blending", config.use_blending);

  // Load SCHUNK powerball specific error codes
  std::string emcy_emergency_errors_filename =
   boost::filesystem::path(std::getenv("CANOPEN_RESOURCE_PATH") /
   boost::filesystem::path("EMCY_schunk.ini")).string();
  EMCY::addEmergencyErrorMap( emcy_emergency_errors_filename, "schunk_error_codes");

  // Create a canopen controller
  m_controller = boost::make_shared<CanOpenController>(can_device_name);

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
    ROS_INFO_STREAM ("Found chain with name " << name << " and " << chain.size() << " nodes");
    chain_configuratuions[name] = chain;
    for (size_t j = 0; j < chain.size(); ++j)
    {
      ROS_INFO_STREAM ("Node " << chain[j]);
      m_controller->addNode<SchunkPowerBallNode>(chain[j], chain_names[i]);

      std::string joint_name = "";
      std::string mapping_key = "~node_mapping_" + boost::lexical_cast<std::string>( chain[j]);
      ros::param::get(mapping_key, joint_name);

      m_joint_name_mapping[joint_name] =  static_cast<uint8_t>(chain[j]);
      joint_msg.name.push_back(joint_name);
    }
  }


  for (size_t i = 0; i < m_chain_handles.size(); ++i)
  {
    m_chain_handles[i]->setupProfilePositionMode(config);
  }

  // initialize all nodes, by default this will start ProfilePosition mode, so we're good to enable nodes
  m_controller->initNodes();

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
  for (size_t i = 0; i < m_chain_handles.size(); ++i)
  {
    try {
      m_chain_handles[i]->enableNodes(mode);
    }
    catch (const ProtocolException& e)
    {
      ROS_ERROR_STREAM ("Caught ProtocolException while enabling nodes from chain " <<
        chain_names[i] << ". Nodes from this group won't be enabled.");
      continue;
    }
    ROS_INFO_STREAM ("Enabled nodes from chain " << chain_names[i]);
  }

  if (m_use_ros_control)
  {
    m_ros_control_thread = boost::thread(&SchunkCanopenNode::rosControlLoop, this);
  }
  else
  {
    m_action_server.start();
  }

  // Create joint state publisher
  m_joint_pub = m_pub_nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  // services
  m_enable_service =  m_pub_nh.advertiseService("enable_nodes", &SchunkCanopenNode::enableNodes, this);
  m_close_brakes_service =  m_pub_nh.advertiseService("close_brakes",
     &SchunkCanopenNode::closeBrakes, this);
  m_quick_stop_service =  m_pub_nh.advertiseService("quick_stop_nodes",
     &SchunkCanopenNode::quickStopNodes, this);
  m_home_service_all = m_pub_nh.advertiseService("home_reset_offset_all",
     &SchunkCanopenNode::homeAllNodes, this);
  m_home_service_joint_names = m_pub_nh.advertiseService("home_reset_offset_by_id",
     &SchunkCanopenNode::homeNodesCanIds, this);
  m_home_service_canopen_ids = m_pub_nh.advertiseService("home_reset_offset_by_name",
     &SchunkCanopenNode::homeNodesJointNames, this);

  ros::Rate loop_rate(frequency);

  DS402Node::Ptr node;
  while (ros::ok())
  {
    ros::spinOnce();

    joint_msg.position.clear();
    for (std::map<std::string, uint8_t>::iterator it = m_joint_name_mapping.begin();
         it != m_joint_name_mapping.end();
         ++it)
    {
      const uint8_t& nr = it->second;
      node = m_controller->getNode<DS402Node>(nr);
      joint_msg.position.push_back(node->getTargetFeedback());
    }
    joint_msg.header.stamp = ros::Time::now();
    m_joint_pub.publish(joint_msg);
    loop_rate.sleep();
  }
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

  SchunkPowerBallNode::Ptr node = m_controller->getNode<SchunkPowerBallNode>(8);

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
    if (m_hardware_interface->isFault())
    {
      ROS_ERROR ("Some nodes are in FAULT state! No output will be sent. Once the fault is removed, call the enable_nodes service.");
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
        m_hardware_interface->write();
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
