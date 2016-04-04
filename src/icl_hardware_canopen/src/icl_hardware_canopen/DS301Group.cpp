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
 * \author  Georg Heppner <heppner@fzi.de>
 * \author  Felix Mauch <mauch@fzi.de>
 * \date    2015-10-1
 *
 */
//----------------------------------------------------------------------

#include "DS301Group.h"

namespace icl_hardware {
namespace canopen_schunk {

DS301Group::DS301Group(const std::string& name)
  : m_name(name)
{
}

void DS301Group::deleteNodes(std::vector<uint8_t>& deleted_ids)
{
  deleted_ids.clear();
  for (size_t i=0; i < m_nodes.size(); ++i)
  {
    deleted_ids.push_back(m_nodes.at(i)->getNodeId());
  }
  m_nodes.clear();
}

bool DS301Group::deleteNodeFromId(const uint8_t node_id)
{
  for (std::vector<DS301Node::Ptr>::iterator it = m_nodes.begin(); it != m_nodes.end(); ++it)
  {
    if ((*it)->getNodeId() == node_id)
    {
      m_nodes.erase(it);
      return true;
    }
  }
  return false;
}

void DS301Group::initPDOMappingSingle (const PDO::MappingConfigurationList& config,
                                       const uint16_t pdo_nr,
                                       const PDO::eTransmissionType& transmission_type,
                                       const DS301Node::ePDO_TYPE& pdo_type,
                                       const bool dummy_mapping,
                                       const uint8_t cyclic_timeout_cycles,
                                       const int16_t node_id)
{
  for (std::vector<DS301Node::Ptr>::iterator it = m_nodes.begin(); it != m_nodes.end(); ++it)
  {
    if ((*it)->getNodeId() == node_id || node_id < 0)
    {
      (*it)->initPDOMappingSingle(config,
                                  pdo_nr,
                                  transmission_type,
                                  pdo_type,
                                  dummy_mapping,
                                  cyclic_timeout_cycles);
    }
  }
}

void DS301Group::appendPDOMappingSingle (const PDO::MappingConfigurationList& config,
                                         const uint16_t pdo_nr,
                                         const PDO::eTransmissionType& transmission_type,
                                         const DS301Node::ePDO_TYPE& pdo_type,
                                         const bool dummy_mapping,
                                         const uint8_t cyclic_timeout_cycles,
                                         const int16_t node_id)
{
  for (std::vector<DS301Node::Ptr>::iterator it = m_nodes.begin(); it != m_nodes.end(); ++it)
  {
    if ((*it)->getNodeId() == node_id || node_id < 0)
    {
      (*it)->appendPDOMappingSingle(config,
                                  pdo_nr,
                                  transmission_type,
                                  pdo_type,
                                  dummy_mapping,
                                  cyclic_timeout_cycles);
    }
  }
}

void DS301Group::uploadPDOs()
{
  for (std::vector<DS301Node::Ptr>::iterator it = m_nodes.begin(); it != m_nodes.end(); ++it)
  {
    (*it)->uploadPDOs();
  }
}

void DS301Group::downloadPDOs()
{
  for (std::vector<DS301Node::Ptr>::iterator it = m_nodes.begin(); it != m_nodes.end(); ++it)
  {
    (*it)->downloadPDOs();
  }
}

void DS301Group::printPDOMapping (const uint8_t node_id)
{
  for (std::vector<DS301Node::Ptr>::iterator it = m_nodes.begin(); it != m_nodes.end(); ++it)
  {
    if ((*it)->getNodeId() == node_id || node_id < 0)
    {
      (*it)->printPDOMapping();
    }
  }
}

void DS301Group::registerWSBroadcaster(boost::shared_ptr<icl_comm::websocket::WsBroadcaster> broadcaster)
{
  m_ws_broadcaster = broadcaster;
}



}} // end of NS
