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
 * \date    2016-3-24
 *
 */
//----------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <boost/filesystem.hpp>

#include <icl_hardware_canopen/CanOpenController.h>
#include <icl_hardware_canopen/SchunkPowerBallNode.h>

using namespace icl_hardware::canopen_schunk;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_arm_to_zero_node");

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  CanOpenController::Ptr my_controller;

  std::string can_device_name;
  std::vector<std::string> chain_names;
  std::map<std::string, std::vector<int> > chain_configuratuions;
  std::vector<DS402Group::Ptr> chain_handles;
  sensor_msgs::JointState joint_msg;
  ds402::ProfilePositionModeConfiguration ppm_config;
  std::map<std::string, uint8_t> joint_name_mapping;

  priv_nh.getParam("chain_names", chain_names);
  priv_nh.param<std::string>("can_device_name", can_device_name, "auto");
  priv_nh.param<float>  ("ppm_profile_velocity",       ppm_config.profile_velocity,         0.2);
  priv_nh.param<float>  ("ppm_profile_acceleration",   ppm_config.profile_acceleration,     0.2);
  priv_nh.param<bool>   ("ppm_use_relative_targets",   ppm_config.use_relative_targets,     false);
  priv_nh.param<bool>   ("ppm_change_set_immediately", ppm_config.change_set_immediately,   true);
  priv_nh.param<bool>   ("ppm_use_blending",           ppm_config.use_blending,             true);

  // Create a canopen controller
  try
  {
    my_controller = boost::make_shared<CanOpenController>(can_device_name);
  }
  catch (const DeviceException& e)
  {
    ROS_ERROR_STREAM ("Initializing CAN device failed. Reason: " << e.what());
    ROS_INFO ("Shutting down now.");
    return -1;
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

  // parse the robot configuration
  for (size_t i = 0; i < chain_names.size(); ++i)
  {
    std::string name = "chain_" + chain_names[i];
    my_controller->addGroup<DS402Group>(chain_names[i]);
    chain_handles.push_back(my_controller->getGroup<DS402Group>(chain_names[i]));
    std::vector<int> chain;
    try
    {
      priv_nh.getParam(name, chain);
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
      my_controller->addNode<SchunkPowerBallNode>(chain[j], chain_names[i]);

      std::string joint_name = "";
      std::string mapping_key = "~node_mapping_" + boost::lexical_cast<std::string>( chain[j]);
      ros::param::get(mapping_key, joint_name);

      joint_name_mapping[joint_name] =  static_cast<uint8_t>(chain[j]);
      joint_msg.name.push_back(joint_name);
    }
  }

  // initialize all nodes, by default this will start ProfilePosition mode, so we're good to enable nodes
  try {
    my_controller->initNodes();
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

  for (size_t i = 0; i < chain_handles.size(); ++i)
  {
    try {
      ROS_INFO_STREAM ("Setting up Profile Position mode for chain " << chain_handles[i]->getName());
      chain_handles[i]->setupProfilePositionMode(ppm_config);
      chain_handles[i]->enableNodes(mode);
    }
    catch (const ProtocolException& e)
    {
      ROS_ERROR_STREAM ("Caught ProtocolException while enabling nodes from chain " <<
        chain_handles[i]->getName() << ". Nodes from this group won't be enabled.");
      continue;
    }
    ROS_INFO_STREAM ("Enabled nodes from chain " << chain_handles[i]->getName());
    std::vector<DS301Node::Ptr> nodes = chain_handles[i]->getNodes();
    std::vector<float> targets (nodes.size(), 0.0);

    chain_handles[i]->setTarget(targets);
  }
  my_controller->enablePPMotion();

  std::vector<bool> foo;

  while ( true )
  {
    size_t num_reached = 0;
    try {
      my_controller->syncAll();
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM (e.what());
      return -11;
    }
    usleep(10000);

    for (size_t i = 0; i < chain_handles.size(); ++i)
    {
      if (chain_handles[i]->isTargetReached(foo))
      {
        num_reached++;
      }
    }
    if (num_reached == chain_handles.size())
    {
      break;
    }
  }

  LOGGING_INFO (CanOpen, "All targets reached" << endl);

  try
  {
    for (size_t i = 0; i < chain_handles.size(); ++i)
    {
      chain_handles[i]->disableNodes();
    }
  }
  catch (const ProtocolException& e)
  {
    ROS_ERROR_STREAM ( "Error while disabling nodes: " << e.what());
  }

  return 0;
}
