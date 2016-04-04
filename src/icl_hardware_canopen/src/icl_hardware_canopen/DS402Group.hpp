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
 * \date    2015-12-01
 *
 */
//----------------------------------------------------------------------

#ifndef DS402GROUP_HPP
#define DS402GROUP_HPP

namespace icl_hardware {
namespace canopen_schunk {

template <typename NodeT>
DS301Node::Ptr DS402Group::addNode (const uint8_t node_id, const CanDevPtr can_device, HeartBeatMonitor::Ptr heartbeat_monitor)
{
  LOGGING_INFO(CanOpen, "Adding new DS402Node with id " << node_id << endl);
  DS301Node::Ptr node(new NodeT(node_id, can_device, heartbeat_monitor));
  DS402Node::Ptr node_ds402 = boost::dynamic_pointer_cast<DS402Node>(node);
  m_nodes.push_back(node);
  m_ds402_nodes.push_back(node_ds402);
  return node;
}

}} //End of NS

#endif
