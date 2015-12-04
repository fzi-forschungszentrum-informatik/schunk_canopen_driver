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



#include "schunk_canopen_driver/SchunkCanopenNode.h"
#include <icl_hardware_canopen/SchunkPowerBallNode.h>
#include "icl_core_logging/Logging.h"



SchunkCanopenNode::SchunkCanopenNode()
  : m_priv_nh("~"),
    m_action_server(m_priv_nh, "follow_joint_trajectory",
      boost::bind(&SchunkCanopenNode::goalCB, this, _1),
      boost::bind(&SchunkCanopenNode::cancelCB, this, _1), false)
{
  std::string can_device_name;
  uint8_t first_node;
  int frequency = 30;
  std::vector<std::string> chain_names;
  std::map<std::string, std::vector<int> > chain_configuratuions;
  m_chain_handles.clear();

  ds402::ProfilePositionModeConfiguration config;
  config.profile_acceleration = 0.2;
  config.profile_velocity = 0.2;
  config.use_relative_targets = false;
  config.change_set_immediately = true;
  config.use_blending = true;


  try
  {
    m_priv_nh.param<std::string>("can_device_name", can_device_name, "/dev/pcanusb1");
    m_priv_nh.getParam("chain_names", chain_names);
  }
  catch (ros::InvalidNameException e)
  {
    ROS_ERROR("Parameter Error! ");
  }

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
    }
  }


  for (size_t i = 0; i < m_chain_handles.size(); ++i)
  {
    m_chain_handles[i]->setupProfilePositionMode(config);
  }

  // initialize all nodes, by default this will start ProfilePosition mode, so we're good to enable nodes
  m_controller->initNodes();

  // Start interface (either action server or ros_control)
  for (size_t i = 0; i < m_chain_handles.size(); ++i)
  {
    try {
      m_chain_handles[i]->enableNodes();
    }
    catch (const ProtocolException& e)
    {
      ROS_ERROR_STREAM ("Caught ProtocolException while enabling nodes from chain " <<
        chain_names[i] << ". Nodes from this group won't be enabled.");
      continue;
    }
    ROS_INFO_STREAM ("Enbaled nodes from chain " << chain_names[i]);
  }
  m_action_server.start();

  // Create joint state publisher

  ros::Rate loop_rate(frequency);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
}

void SchunkCanopenNode::goalCB (actionlib::ServerGoalHandle< control_msgs::FollowJointTrajectoryAction > gh)
{
  ROS_INFO ("Executing Trajectory action");
  gh.setAccepted();
  control_msgs::FollowJointTrajectoryActionResult result;
  std::stringstream joints, positions;
  if (gh.getGoal()->trajectory.joint_names.size() != gh.getGoal()->trajectory.points.size())
  {
    ROS_ERROR ("Number if given joint names and joint states do not match! Aborting goal!");
    return;
  }

  for (size_t i = 0; i < gh.getGoal()->trajectory.joint_names.size(); ++i)
  {
    joints << gh.getGoal()->trajectory.joint_names[i] << " ";
    positions << gh.getGoal()->trajectory.points[i].positions[0] << " ";

    uint8_t nr = boost::lexical_cast<int>(gh.getGoal()->trajectory.joint_names[i]);
    float pos = boost::lexical_cast<float>(gh.getGoal()->trajectory.points[i].positions[0]);
    ROS_INFO_STREAM ("Joint " << static_cast<int>(nr) << ": " << pos);
    SchunkPowerBallNode::Ptr node = m_controller->getNode<SchunkPowerBallNode>(nr);
    m_controller->getNode<SchunkPowerBallNode>(nr)->setTarget(pos);
  }
  DS402Group::Ptr grp = m_controller->getGroup<DS402Group>("arm");

  ros::Time start = ros::Time::now();
  ros::Duration max_time = gh.getGoal()->goal_time_tolerance;

  ros::Duration spent_time = start - start;
  std::vector<bool> reached_vec;

  // Give the brakes time to open up
  sleep(1);

  while (spent_time < max_time)
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

    bool targets_reached = true;
    for (size_t i = 0; i < gh.getGoal()->trajectory.joint_names.size(); ++i)
    {
      uint8_t nr = boost::lexical_cast<int>(gh.getGoal()->trajectory.joint_names[i]);
      SchunkPowerBallNode::Ptr node = m_controller->getNode<SchunkPowerBallNode>(nr);
      targets_reached &= node->isTargetReached();
    }

    if (targets_reached)
    {
      ROS_INFO ("All targets reached" );
      gh.setSucceeded();
      break;
    }
    spent_time = ros::Time::now() - start;
    if (spent_time > max_time)
    {
      gh.setAborted();
      break;
    }
  }

}

void SchunkCanopenNode::cancelCB (actionlib::ServerGoalHandle< control_msgs::FollowJointTrajectoryAction > gh)
{
  ROS_INFO ("Canceling Trajectory action");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "schunk_chain");
  icl_core::logging::initialize(argc, argv);

  SchunkCanopenNode my_schunk_node;

  return 0;
}
