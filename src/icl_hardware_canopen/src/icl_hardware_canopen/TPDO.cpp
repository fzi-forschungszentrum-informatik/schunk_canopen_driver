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

#include "TPDO.h"

#include "ds301.h"
#include "Logging.h"
#include "exceptions.h"

namespace icl_hardware {
namespace canopen_schunk {

TPDO::TPDO(const uint8_t node_id, const uint8_t pdo_nr, const CanDevPtr& can_device)
  : PDO (node_id, pdo_nr, can_device),
    m_data_update_received(false)
{
}

void TPDO::update (const CanMsg& msg)
{
  uint8_t node_id = msg.id - ds301::ID_TPDO1_MIN + 1 + (m_pdo_nr << 8);

  if (node_id != m_node_id)
  {
    std::stringstream ss;
    ss << "PDO Update called with wrong canopen ID. Received ID: " << node_id << " Node ID: " << m_node_id << ". Update ignored.";
    throw PDOException (ss.str());
  }

  boost::mutex::scoped_lock(m_data_buffer_mutex);


//   if (m_data_update_received)
//   {
//     LOGGING_WARNING_C (CanOpen, TPDO, "Data buffer contains unprocessed data which will be overwritten." << endl);
//   }

  m_data_buffer.resize(msg.dlc);
  // To increase processing times of PDOs we use a memcopy instead of a loop
  std::memcpy (&m_data_buffer[0], msg.data, msg.dlc);

  uint8_t byte_offset = 0;
  for (MappingList::iterator it = m_mapping_list.begin();
       it != m_mapping_list.end();
       ++it)
  {
    // Copy subset of vector
    std::copy(m_data_buffer.begin() + byte_offset,
              m_data_buffer.begin() + byte_offset + it->data.size(),
              it->data.begin());
    byte_offset += it->data.size();
  }

//   m_data_update_received = true;
  m_data_buffer_updated_cond.notify_one();
}

void TPDO::upload()
{
  for (size_t i = 0; i < m_notify_callbacks.size(); ++i)
  {
    m_notify_callbacks[i]();
  }
}

PDO::PDOStringMatchVec TPDO::remap (SDO& sdo,
                                    const PDO::MappingConfigurationList& mappings,
                                    const PDO::eTransmissionType& transmission_type,
                                    const bool dummy_mapping,
                                    const uint8_t cyclic_timeout_cycles)
{
  uint16_t pdo_cob_id = ds301::ID_TPDO1_MIN + (m_pdo_nr << 8) + m_node_id -1;
  uint16_t pdo_communication_parameter = OD_TPDO_COMMUNICATION_MIN + m_pdo_nr;
  uint16_t pdo_mapping_parameter = OD_TPDO_MAPPING_PARAMETER_MIN + m_pdo_nr;

  return PDO::remap(sdo,
                    mappings,
                    transmission_type,
                    pdo_cob_id,
                    pdo_communication_parameter,
                    pdo_mapping_parameter,
                    dummy_mapping,
                    cyclic_timeout_cycles);
}

PDO::PDOStringMatchVec TPDO::appendMapping (SDO& sdo,
                                            const PDO::MappingConfigurationList& mappings,
                                            const PDO::eTransmissionType& transmission_type,
                                            const bool dummy_mapping,
                                            const uint8_t cyclic_timeout_cycles)
{
  uint16_t pdo_cob_id = ds301::ID_TPDO1_MIN + (m_pdo_nr << 8) + m_node_id -1;
  uint16_t pdo_communication_parameter = OD_TPDO_COMMUNICATION_MIN + m_pdo_nr;
  uint16_t pdo_mapping_parameter = OD_TPDO_MAPPING_PARAMETER_MIN + m_pdo_nr;

  return PDO::appendMapping(sdo,
                            mappings,
                            transmission_type,
                            pdo_cob_id,
                            pdo_communication_parameter,
                            pdo_mapping_parameter,
                            dummy_mapping,
                            cyclic_timeout_cycles);
}


void TPDO::registerNotifyCallback (const boost::function< void() >& f)
{
  m_notify_callbacks.push_back(f);
}



}} // end of NS
