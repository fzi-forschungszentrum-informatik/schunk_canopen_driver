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

#include "DS301Node.h"
#include "exceptions.h"

namespace icl_hardware {
namespace canopen_schunk {

DS301Node::DS301Node(const uint8_t node_id, const CanDevPtr& can_device, HeartBeatMonitor::Ptr heartbeat_monitor):
  m_nmt(node_id,can_device),
  m_sdo(node_id, can_device),
  m_emcy(EMCY::Ptr(new EMCY(node_id))),
  m_node_id(node_id),
  m_can_dev(can_device),
  m_heartbeat_monitor(heartbeat_monitor),
  m_heartbeat_cycle_time_ms(50)
{
  for (size_t i = 0; i < 4; ++i)
  {
    m_rpdos.push_back(RPDO::Ptr(new RPDO(node_id, i, can_device)));
    m_tpdos.push_back(TPDO::Ptr(new TPDO(node_id, i, can_device)));
  }
}

void DS301Node::initNode()
{
  m_nmt.preOperational();
  startHeartbeat();
}

void DS301Node::registerWSBroadcaster(boost::shared_ptr<icl_comm::websocket::WsBroadcaster> broadcaster)
{
  m_ws_broadcaster = broadcaster;
}

void DS301Node::startHeartbeat()
{
  m_sdo.download(false, 0x1017, 0, m_heartbeat_cycle_time_ms);
  // add initial heartbeat. If the device never sends a heartbeat the monitor will escalate quickly
  m_heartbeat_monitor->addHeartbeat(m_node_id);
}


void DS301Node::initPDOMappingSingle (const PDO::MappingConfigurationList& config,
                                      const uint16_t pdo_nr,
                                      const PDO::eTransmissionType& transmission_type,
                                      const ePDO_TYPE& direction_type,
                                      const bool dummy_mapping,
                                      const uint8_t cyclic_timeout_cycles)
{
  PDO::PDOStringMatchVec mapping;
  boost::unordered_map<std::string, PDOMapEntry>* map;
  if (direction_type == RECEIVE_PDO)
  {
    mapping = m_rpdos.at(pdo_nr)->remap(m_sdo, config, transmission_type, dummy_mapping, cyclic_timeout_cycles);
    map = &m_rpdo_mapping;
  }
  else if (direction_type == TRANSMIT_PDO)
  {
    mapping = m_tpdos.at(pdo_nr)->remap(m_sdo, config, transmission_type, dummy_mapping, cyclic_timeout_cycles);
    map = &m_tpdo_mapping;
  }
  else
  {
    LOGGING_ERROR_C (CanOpen, DS301Node, "Illegal PDO type given. Only RECEIVE_PDO and TRANSMIT_PDO are allowed." << endl);
  }

  // remove all mapping entries from this pdo
  for (boost::unordered_map<std::string, PDOMapEntry>::iterator it = map->begin();
       it != map->end();
       /* iterating is done in the body */)
  {
    if (it->second.pdo_nr == pdo_nr)
    {
      map->erase(it++);
    }
    else
    {
      ++it;
    }
  }

  // Now add the configured pdos to the node's mapping
  for (PDO::PDOStringMatchVec::iterator it = mapping.begin(); it != mapping.end(); ++it)
  {
    PDOMapEntry entry;
    entry.pdo_nr = pdo_nr;
    entry.pdo_mapping_index = it->pdo_mapping_index;

    map->insert(std::pair<std::string, PDOMapEntry>(it->name, entry));
  }
}

void DS301Node::appendPDOMappingSingle (const PDO::MappingConfigurationList& config,
                                       const uint16_t pdo_nr,
                                       const PDO::eTransmissionType& transmission_type,
                                       const DS301Node::ePDO_TYPE& direction_type,
                                       const bool dummy_mapping,
                                       const uint8_t cyclic_timeout_cycles)
{
  PDO::PDOStringMatchVec mapping;
  boost::unordered_map<std::string, PDOMapEntry>* map;

  if (direction_type == RECEIVE_PDO)
  {
    mapping = m_rpdos.at(pdo_nr)->appendMapping(m_sdo, config, transmission_type, dummy_mapping, cyclic_timeout_cycles);
    map = &m_rpdo_mapping;
  }
  else if (direction_type == TRANSMIT_PDO)
  {
    mapping = m_tpdos.at(pdo_nr)->appendMapping(m_sdo, config, transmission_type, dummy_mapping, cyclic_timeout_cycles);
    map = &m_tpdo_mapping;
  }
  else
  {
    LOGGING_ERROR_C (CanOpen, DS301Node, "Illegal PDO type given. Only RECEIVE_PDO and TRANSMIT_PDO are allowed." << endl);
  }

  // Now add the configured pdos to the node's mapping
  for (PDO::PDOStringMatchVec::iterator it = mapping.begin(); it != mapping.end(); ++it)
  {
    PDOMapEntry entry;
    entry.pdo_nr = pdo_nr;
    entry.pdo_mapping_index = it->pdo_mapping_index;

    map->insert(std::pair<std::string, PDOMapEntry>(it->name, entry));
  }
}



void DS301Node::uploadPDOs()
{
  for (TPDO::PtrList::iterator it = m_tpdos.begin(); it != m_tpdos.end(); ++it)
  {
    (*it)->upload();
  }
}

void DS301Node::downloadPDOs()
{
  for (RPDO::PtrList::iterator it = m_rpdos.begin(); it != m_rpdos.end(); ++it)
  {
    (*it)->download();
  }
}

void DS301Node::printPDOMapping()
{
  uint32_t data32;
  size_t max_num_rpdo = 512;
  // get number of RPDOs

  std::stringstream ss;
  ss << "PDO Mapping queried from device:" << std::endl;
  ss << "===== RPDOs ===== " << std::endl;

  for (uint8_t i = 0; i < max_num_rpdo; ++i)
  {
    uint8_t num_entries;
    try
    {
      m_sdo.upload(false, 0x1600+i, 0, num_entries);
    }
    catch (const ProtocolException& e)
    {
      // This PDO object does not exist
      break;
    }
    ss << "  === RPDO " << static_cast<int>(i) << " - " << static_cast<int>(num_entries) << " entries ===" << std::endl;

    for (uint8_t j = 1; j <= num_entries; ++j)
    {
      m_sdo.upload(false, 0x1600+i, j, data32);
      int index = (data32 & 0xFFFF0000) >> 16;
      int subindex = (data32 & 0x0000FF00) >> 8;
      int length = data32 & 0x000000FF;
      ss << "    " << static_cast<int>(j) << " -> "
         << hexToString(index) << " / " << subindex << ", length: " << length << " bits" << std::endl;
    }
  }

  ss << std::endl;

  size_t max_num_tpdo = 512;

  ss << "===== TPDOs ===== " << std::endl;

  for (uint8_t i = 0; i < max_num_tpdo; ++i)
  {
    uint8_t num_entries;
    try
    {
      m_sdo.upload(false, 0x1A00+i, 0, num_entries);
    }
    catch (const ProtocolException& e)
    {
      // This PDO object does not exist
      break;
    }
    ss << "  === TPDO " << static_cast<int>(i) << " - " << static_cast<int>(num_entries) << " entries ===" << std::endl;

    for (uint8_t j = 1; j <= num_entries; ++j)
    {
      m_sdo.upload(false, 0x1A00+i, j, data32);
      int index = (data32 & 0xFFFF0000) >> 16;
      int subindex = (data32 & 0x0000FF00) >> 8;
      int length = data32 & 0x000000FF;
      ss << "    " << static_cast<int>(j) << " -> "
         << hexToString(index) << " / " << subindex << ", length: " << length << " bits" << std::endl;
    }
  }

  LOGGING_INFO (CanOpen, ss.str() << endl);
}

void DS301Node::registerPDONotifyCallback (const std::string& identifier,
                                           const boost::function< void() >& f)
{
  boost::unordered_map<std::string,PDOMapEntry>::iterator it = m_tpdo_mapping.find(identifier);
  if(it != m_tpdo_mapping.end())
  {
     size_t pdo_nr = it->second.pdo_nr;
     m_tpdos[pdo_nr]->registerNotifyCallback(f);
     LOGGING_DEBUG_C (CanOpen, DS302Node, "Registered notification callback for PDO entry " <<
                            identifier << endl;
    );
  }
  else
  {
    std::stringstream ss;
    ss << "Notifier callback function for a PDO entry named " << identifier
       << " requested, however, no entry with this given identifier exists within this PDO";
    throw PDOException(ss.str());
  }
}

void DS301Node::stopNode()
{
  m_nmt.stop();
}



}}// end of NS
