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
* \date    2015-10-23
*
*/
//----------------------------------------------------------------------

#include "RPDO.h"

#include "ds301.h"
#include "Logging.h"
#include "exceptions.h"


namespace icl_hardware {
namespace canopen_schunk {


RPDO::RPDO(const uint8_t node_id, const uint8_t pdo_nr, const CanDevPtr& can_device)
  : PDO(node_id, pdo_nr, can_device)
{

}

bool RPDO::download()
{
  if (m_mapping_list.size() == 0)
  {
    // Nothing to do here...
    return true;
  }
  CanMsg msg;
  msg.id = ds301::ID_RPDO1_MIN + m_node_id - 1 + (m_pdo_nr << 8);
  msg.dlc = 8;
  msg.rtr = 0;

  uint8_t byte_offset = 0;
  for (MappingList::iterator it = m_mapping_list.begin();
       it != m_mapping_list.end();
       ++it)
  {
    if (byte_offset + it->data.size() > 8)
    {
      throw PDOException ("Too much data for one PDO transmission. The PDO mapping contains too much data! Aborting download.");
    }

    // Copy subset of vector
    std::copy(it->data.begin(),
              it->data.begin() + it->data.size(),
              msg.data + byte_offset);
    byte_offset += it->data.size();
  }

  LOGGING_TRACE (CanOpen, "Now sending RPDO download can message with id " << msg.id <<
                 ",length " << byte_offset << ", data " << hexArrayToString(msg.data, byte_offset) <<
                 " to node " << m_node_id << endl);
  msg.dlc = byte_offset;
  m_can_device->Send(msg);

  return true;
}

PDO::PDOStringMatchVec RPDO::remap (SDO& sdo,
                                    const MappingConfigurationList& mappings,
                                    const eTransmissionType& transmission_type,
                                    const bool dummy_mapping,
                                    const uint8_t cyclic_timeout_cycles
                                   )
{
  uint16_t pdo_cob_id = ds301::ID_RPDO1_MIN + (m_pdo_nr << 8) + m_node_id - 1;
  uint16_t pdo_communication_parameter = OD_RPDO_COMMUNICATION_MIN + m_pdo_nr;
  uint16_t pdo_mapping_parameter = OD_RPDO_MAPPING_PARAMETER_MIN + m_pdo_nr;

  return PDO::remap(sdo,
                    mappings,
                    transmission_type,
                    pdo_cob_id,
                    pdo_communication_parameter,
                    pdo_mapping_parameter,
                    dummy_mapping,
                    cyclic_timeout_cycles);
}


PDO::PDOStringMatchVec RPDO::appendMapping (SDO& sdo,
                                            const MappingConfigurationList& mappings,
                                            const eTransmissionType& transmission_type,
                                            const bool dummy_mapping,
                                            const uint8_t cyclic_timeout_cycles
                             )
{
  uint16_t pdo_cob_id = ds301::ID_RPDO1_MIN + (m_pdo_nr << 8) + m_node_id - 1;
  uint16_t pdo_communication_parameter = OD_RPDO_COMMUNICATION_MIN + m_pdo_nr;
  uint16_t pdo_mapping_parameter = OD_RPDO_MAPPING_PARAMETER_MIN + m_pdo_nr;

  return PDO::appendMapping(sdo,
                            mappings,
                            transmission_type,
                            pdo_cob_id,
                            pdo_communication_parameter,
                            pdo_mapping_parameter,
                            dummy_mapping,
                            cyclic_timeout_cycles);
}


}} // end of NS
