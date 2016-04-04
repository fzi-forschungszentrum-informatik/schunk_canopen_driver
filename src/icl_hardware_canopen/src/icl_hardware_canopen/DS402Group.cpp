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
 * \author  Georg Heppner <heppner@fzi.de>
 * \date    2015-10-1
 *
 */
//----------------------------------------------------------------------

#include "DS402Group.h"

namespace icl_hardware {
namespace canopen_schunk {

DS402Group::DS402Group(const std::string& name)
 : DS301Group(name)
{
}

void DS402Group::initNodes(const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->initNode();
    }
  }
}


void DS402Group::printStatus(const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->printStatus();
    }
  }
}

bool DS402Group::setTarget (const float target, const uint8_t node_id)
{
  bool successful = false;
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id)
    {
      successful = node->setTarget(target);
    }
  }

  return successful;
}

bool DS402Group::setTarget (const std::vector< float >& targets)
{
  bool all_successful = true;
  size_t i = 0;
  if (targets.size() != m_ds402_nodes.size())
  {
    LOGGING_ERROR (CanOpen, "The given number of target points (" << targets.size() <<
      ") does not match the " <<
      "number of nodes registered to this group (" << m_ds402_nodes.size() << ")." << endl
    );
    return false;
  }

  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    all_successful &= node->setTarget(targets[i]);
    ++i;
  }

  return all_successful;
}

void DS402Group::startPPMovement(const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->startPPMovement();
    }
  }
}

void DS402Group::acceptPPTargets(const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->acceptPPTargets();
    }
  }
}

void DS402Group::getTargetFeedback (std::vector< double >& feedback)
{
  feedback.resize(m_ds402_nodes.size());
  size_t i = 0;
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    feedback[i] = node->getTargetFeedback();
    ++i;
  }
}

void DS402Group::setDefaultPDOMapping (const DS402Node::eDefaultPDOMapping mapping, const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->setDefaultPDOMapping(mapping);
    }
  }
}

void DS402Group::home (const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->home();
    }
  }
}

void DS402Group::getModeOfOperation (std::vector< ds402::eModeOfOperation >& modes)
{
  modes.resize(m_ds402_nodes.size());
  size_t i = 0;
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    modes[i] = node->getModeOfOperation();
    ++i;
  }
}

bool DS402Group::setModeOfOperation (const ds402::eModeOfOperation op_mode, const int16_t node_id)
{
  bool all_successful = true;
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      all_successful &= node->setModeOfOperation(op_mode);
    }
  }

  return all_successful;
}

bool DS402Group::isModeSupported (const ds402::eModeOfOperation op_mode, const int16_t node_id)
{
  bool supported_by_all = true;
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      supported_by_all &= node->isModeSupported(op_mode);
    }
  }

  return supported_by_all;
}

void DS402Group::quickStop (const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->quickStop();
      #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
      if (m_ws_broadcaster)
      {
        m_ws_broadcaster->robot->setJointEnabled(false, node->getNodeId());
      }
      #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_
    }
  }
}



void DS402Group::setupProfilePositionMode (const ds402::ProfilePositionModeConfiguration& config, const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->setupProfilePositionMode(config);
    }
  }
}

void DS402Group::setupHomingMode (const ds402::HomingModeConfiguration& config, const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->setupHomingMode(config);
    }
  }
}

void DS402Group::setupProfileVelocityMode (const ds402::ProfileVelocityModeConfiguration& config, const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->setupProfileVelocityMode(config);
    }
  }
}

void DS402Group::setupProfileTorqueMode (const ds402::ProfileTorqueModeConfiguration& config, const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->setupProfileTorqueMode(config);
    }
  }
}

bool DS402Group::resetFault (const int16_t node_id)
{
  bool all_successful = true;
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      all_successful &= node->resetFault();
    }
  }

  return all_successful;
}

void DS402Group::configureInterpolationCycleTime (const int16_t node_id, const uint8_t interpolation_cycle_time_ms)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->configureInterpolationCycleTime(interpolation_cycle_time_ms);
    }
  }
}

void DS402Group::configureHomingSpeeds (const uint32_t low_speed, const uint32_t high_speed, const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->configureHomingSpeeds(low_speed, high_speed);
    }
  }
}

void DS402Group::configureHomingMethod (const int8_t homing_method, const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->configureHomingSpeeds(homing_method);
    }
  }
}

void DS402Group::enableNodes (const ds402::eModeOfOperation operation_mode, const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->enableNode(operation_mode);

      #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
      if (m_ws_broadcaster)
      {
        m_ws_broadcaster->robot->setJointEnabled(true, node->getNodeId());
      }
      #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_

    }
  }
}

void DS402Group::disableNodes (const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->disableNode();

      #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
      if (m_ws_broadcaster)
      {
        m_ws_broadcaster->robot->setJointEnabled(false, node->getNodeId());
      }
      #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_
    }
  }
}

void DS402Group::openBrakes (const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->openBrakes();
      #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
      if (m_ws_broadcaster)
      {
        m_ws_broadcaster->robot->setJointEnabled(true, node->getNodeId());
      }
      #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_
    }
  }
}

void DS402Group::closeBrakes (const int16_t node_id)
{
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    if (node->getNodeId() == node_id || node_id < 0)
    {
      node->closeBrakes();
      #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
      if (m_ws_broadcaster)
      {
        m_ws_broadcaster->robot->setJointEnabled(false, node->getNodeId());
      }
      #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_
    }
  }
}

bool DS402Group::isTargetReached (std::vector< bool >& reached_single)
{
  reached_single.resize(m_ds402_nodes.size());
  bool reached_all = true;
  size_t i = 0;
  for (std::vector<DS402Node::Ptr>::iterator it = m_ds402_nodes.begin();
       it != m_ds402_nodes.end();
       ++it)
  {
    DS402Node::Ptr node = *it;
    reached_single[i] = node->isTargetReached();
    reached_all &= reached_single[i];
    ++i;
  }

  return reached_all;
}





}} // end of NS
