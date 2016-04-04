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
 * \date    2015-10-8
 *
 */
//----------------------------------------------------------------------
#ifndef CANOPENCONTROLLER_HPP
#define CANOPENCONTROLLER_HPP

#include "Logging.h"
#include "exceptions.h"
#include "DS402Group.h"

namespace icl_hardware {
namespace canopen_schunk {

template <typename GroupT>
void CanOpenController::addGroup(const std::string& identifier)
{
  std::string sanitized_identifier = sanitizeString(identifier);
  if (m_groups.find(sanitized_identifier) == m_groups.end())
  {
    DS301Group::Ptr group(new GroupT(sanitized_identifier));

    #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
    group->registerWSBroadcaster(m_ws_broadcaster);
    #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_

    m_groups[sanitized_identifier] = group;

  }
  else
  {
    LOGGING_ERROR_C(CanOpen, CanOpenController, "Group with the given identifier " << sanitized_identifier
      << " already exists. Not adding new group." << endl);
  }
}

template <typename GroupT>
boost::shared_ptr<GroupT> CanOpenController::getGroup (const std::string& index)
{
    std::string sanitized_index = sanitizeString(index);
    boost::shared_ptr<GroupT> group;
    if (m_groups.find(sanitized_index) != m_groups.end())
    {
      group = boost::dynamic_pointer_cast<GroupT>(m_groups[sanitized_index]);
      if (!group)
      {
        LOGGING_ERROR_C(CanOpen, CanOpenController, "Cannot cast group to requested type. Returning null pointer." << endl);
      }
    }
    else
    {
      std::stringstream ss;
      ss << "No group with the given index " << sanitized_index
        << " exists. Returning null pointer.";
      throw NotFoundException(ss.str());
    }
    return group;
  }


template <typename NodeT>
void CanOpenController::addNode(const uint8_t node_id, const std::string& group_name)
{
  std::string sanitized_identifier = sanitizeString(group_name);
  std::map<std::string, DS301Group::Ptr>::iterator group_it;
  group_it = m_groups.find(sanitized_identifier);


  if (m_nodes.find(node_id) == m_nodes.end())
  {
    if (group_it != m_groups.end())
    {
      DS301Node::Ptr new_node;

      // TODO: Maybe this can be done prettier with templates, however for now this works
      DS402Group::Ptr ds402_ptr;
      if (ds402_ptr = boost::dynamic_pointer_cast<DS402Group>(group_it->second))
      {
        new_node = ds402_ptr->addNode<NodeT>(node_id, m_can_device, m_heartbeat_monitor);
      }
      else
      {
        new_node = group_it->second->addNode<NodeT>(node_id, m_can_device, m_heartbeat_monitor);
      }

      #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
        new_node->registerWSBroadcaster(m_ws_broadcaster);
      #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_

      m_nodes.insert (std::pair<uint8_t, DS301Node::Ptr>(node_id, new_node));
    }
    else
    {
      LOGGING_ERROR_C(CanOpen, CanOpenController, "No group with the given index " << sanitized_identifier
        << " exists. New node not added!" << endl);
    }
  }
  else
  {
    LOGGING_ERROR_C(CanOpen, CanOpenController, "Node with CANOPEN ID " << node_id
      << " already exists. Not adding new node."  << endl);
  }
}

template <typename NodeT>
boost::shared_ptr<NodeT> CanOpenController::getNode (const uint8_t node_id)
{
  if (m_nodes.find(node_id) == m_nodes.end())
  {
    std::stringstream ss;
    ss << "No node with the given id '" << static_cast<int>(node_id) << "' found.";
    throw NotFoundException(ss.str());
  }

  if (boost::dynamic_pointer_cast<NodeT>(m_nodes[node_id]))
  {
    return boost::dynamic_pointer_cast<NodeT>(m_nodes[node_id]);
  }
  else
  {
    throw std::bad_cast();
  }
}

}} //End of NS

#endif
