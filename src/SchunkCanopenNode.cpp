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


#include "schunk_canopen_driver/SchunkCanopenNode.h"
#include <icl_hardware_canopen/SchunkPowerBallNode.h>
#include "icl_core_logging/Logging.h"



SchunkCanopenNode::SchunkCanopenNode()
  : m_priv_nh("~"),
    m_action_server(m_priv_nh, "follow_joint_trajectory",
    boost::bind(&SchunkCanopenNode::goalCB, this, _1),
    boost::bind(&SchunkCanopenNode::cancelCB, this, _1), false),
    m_has_goal(false),
    m_use_ros_control(false)
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
    ros::param::get("~use_ros_control", m_use_ros_control);
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

  if (m_use_ros_control)
  {
    m_hardware_interface.reset(
      new SchunkCanopenHardwareInterface(m_pub_nh, m_controller));
    m_controller_manager.reset(
      new controller_manager::ControllerManager( m_hardware_interface.get(), m_pub_nh));
  }

  // Start interface (either action server or ros_control)
  for (size_t i = 0; i < m_chain_handles.size(); ++i)
  {
    try {
//       m_chain_handles[i]->enableNodes();
      m_chain_handles[i]->enableNodes(ds402::MOO_INTERPOLATED_POSITION_MODE);
    }
    catch (const ProtocolException& e)
    {
      ROS_ERROR_STREAM ("Caught ProtocolException while enabling nodes from chain " <<
        chain_names[i] << ". Nodes from this group won't be enabled.");
      continue;
    }
    ROS_INFO_STREAM ("Enbaled nodes from chain " << chain_names[i]);
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

  /* TODO: Catch errors:
   * - Joint not enabled
   * - EmergencyStopState
   * - Overwriting trajectory
   * - invalid positions
   */


  if (gh.getGoal()->trajectory.joint_names.size() != gh.getGoal()->trajectory.points.size())
  {
    ROS_ERROR ("Number if given joint names and joint states do not match! Aborting goal!");
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

  for (size_t i = 0; i < gh.getGoal()->trajectory.joint_names.size(); ++i)
  {
    uint8_t nr = boost::lexical_cast<int>(gh.getGoal()->trajectory.joint_names[i]);
    float pos = boost::lexical_cast<float>(gh.getGoal()->trajectory.points[i].positions[0]);
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
  ros::Time start = ros::Time::now();

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

    bool targets_reached = true;
    for (size_t i = 0; i < gh.getGoal()->trajectory.joint_names.size(); ++i)
    {
      uint8_t nr = boost::lexical_cast<int>(gh.getGoal()->trajectory.joint_names[i]);
      SchunkPowerBallNode::Ptr node = m_controller->getNode<SchunkPowerBallNode>(nr);
      targets_reached &= node->isTargetReached();
      float pos = node->getTargetFeedback();
//       ROS_INFO_STREAM ("Node " << nr << " target reached: " << targets_reached <<
//         ", position is: " << pos
//       );
      feedback.feedback.actual.time_from_start = spent_time;
      feedback.feedback.actual.positions.at(i) = (pos);
    }


    gh.publishFeedback(feedback.feedback);

    if (targets_reached)
    {
      ROS_INFO ("All targets reached" );
      result.result.error_code = 0;
      result.result.error_string = "All targets successfully reached";
      gh.setSucceeded(result.result);
      break;
    }
    spent_time = ros::Time::now() - start;
    if (spent_time > max_time && !max_time.isZero())
    {
      result.result.error_code = -5;
      result.result.error_string = "Did not reach targets in specified time";
      gh.setAborted();
      break;
    }
    if (boost::this_thread::interruption_requested())
    {
      ROS_ERROR ("Interruption requested");
      break;
    }
  }
  m_has_goal = false;
}


void SchunkCanopenNode::cancelCB (actionlib::ServerGoalHandle< control_msgs::FollowJointTrajectoryAction > gh)
{
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
  }

  SchunkPowerBallNode::Ptr node = m_controller->getNode<SchunkPowerBallNode>(8);

  size_t counter = 0;

  while (ros::ok()) {
    current_time = ros::Time::now();
    elapsed_time = current_time - last_time;
    last_time = current_time;
    // Input
    m_hardware_interface->read();
    // Control
    m_controller->syncAll();
//     node->printStatus();
    m_controller_manager->update(current_time, elapsed_time);

    // Output
    /* Give the controller some time, otherwise it will send a 0 waypoint vector which
     * might lead into a following error, if joints are not at 0
     * TODO: Find a better solution for that.
     */
    if (counter > 20)
    {
      m_hardware_interface->write();
    }
    ++counter;

    usleep(10000);
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "schunk_chain");
  icl_core::logging::initialize(argc, argv);

  SchunkCanopenNode my_schunk_node;

  return 0;
}
