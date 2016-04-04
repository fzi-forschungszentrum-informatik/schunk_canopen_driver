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

#include "EMCY.h"
#include "ds301.h"

#include "Logging.h"
#include "exceptions.h"

namespace icl_hardware {
namespace canopen_schunk {

// static initializers of error_maps
std::map <uint16_t, std::string> EMCY::m_eec_map;
std::map <uint8_t, std::string> EMCY::m_error_register_map;

EMCY::EMCY(const uint8_t node_id)
  : m_node_id(node_id)
{
}

void EMCY::update(const CanMsg& msg)
{
  uint8_t node_id = msg.id - ds301::ID_EMCY_MIN + 1;

  if (node_id != m_node_id)
  {
    LOGGING_ERROR_C(CanOpen, EMCY, "EMCY Update called with wrong canopen ID. Received ID: "
                                   << node_id << " Node ID: " << m_node_id
                                   << ". Update ignored." << endl);
    return;
  }


  if (msg.dlc != 8)
  {
    std::stringstream ss;
    ss << "Unexpected length " << static_cast<int>(msg.dlc) << " of EMCY message. Expected 8.";
    LOGGING_ERROR_C (CanOpen, SDO, ss.str() << endl);
    return;
  }

  boost::mutex::scoped_lock(m_data_buffer_mutex);
  // isolate information from message
  m_eec = msg.data[0] + (msg.data[1] << 8);
  m_error_register = msg.data[2];
  m_msef.resize(5);
  for (size_t i = 3; i < 8; ++i)
  {
    m_msef[i-3] = msg.data[i];
  }

  if (m_eec == EMCY_ERROR_RESET_NO_ERROR)
  {
    m_error_state = EMCY_STATE_ERROR_FREE;
    LOGGING_INFO_C(CanOpen, EMCY, "Error reset EMCY received. Node " << m_node_id
    << " is now in state error free." << endl);
  }
  else
  {
    m_error_state = canopen_schunk::EMCY::EMCY_STATE_ERROR_OCCURED;
    std::stringstream ss;
    ss << "EMCY message states that an error in node "
       << static_cast<int>(m_node_id) << " occured: "
       << lookupEECString(m_eec) << std::endl
       << "Error registers: " << lookupErrorRegisterString(m_error_register) << std::endl
       << "Manufacturer specific code: " << lookupMSEFString();
    LOGGING_ERROR_C(CanOpen, EMCY, ss.str() << endl);
  }
}

EMCY::eEMCY_STATUS EMCY::getEmcyStatus() const
{
  boost::mutex::scoped_lock(m_data_buffer_mutex);
  return m_error_state;
}

bool EMCY::getErrorInformation(uint16_t& eec, uint8_t& error_register, std::vector< uint8_t >& msef)
{
  boost::mutex::scoped_lock(m_data_buffer_mutex);

  // No error occured, so we can give no valid error information
  if (m_error_state == EMCY_STATE_ERROR_FREE)
  {
    return false;
  }

  eec = m_eec;
  error_register = m_error_register;
  msef = m_msef;
  return true;
}

std::string EMCY::lookupEECString(const uint16_t error_code)
{
  std::stringstream ss;
  ss << std::endl;
  uint16_t nibble_validator = 0xF000;
  uint16_t subcode = error_code & nibble_validator;
  std::map<uint16_t, std::string>::iterator map_it = m_eec_map.find(subcode);
  if (map_it != m_eec_map.end())
  {
    ss << map_it->second << std::endl;
    uint16_t nibble_validator_next = nibble_validator >> 4;
    uint16_t next_nibble = error_code & nibble_validator_next;
    ++map_it;

    // DS402 allows additional functions to be defined directly as 0xF00N
    // (according to table 3 in CiA DSP-402 V1.1)
    while (next_nibble == 0 && nibble_validator_next > 0)
    {
      subcode |= next_nibble;
      nibble_validator += nibble_validator_next;
      nibble_validator_next = nibble_validator_next >> 4;
      next_nibble = error_code & nibble_validator_next;
    }

    // Iterate forward in the map (it is sorted) as long as the code matches
    while ((map_it->first & nibble_validator) == subcode)
    {
      if ( (map_it->first & nibble_validator_next) == next_nibble )
      {
        // found the next nibble. Move everything forward one nibble.
        while ((map_it->first & nibble_validator_next) != 0 && subcode != error_code)
        {
          subcode |= next_nibble;
          nibble_validator += nibble_validator_next;
          nibble_validator_next = nibble_validator_next >> 4;
          next_nibble = error_code & nibble_validator_next;
        }

        if ( (map_it->first - subcode) == 0 )
        {
          // add value to output string
          ss << map_it->second << std::endl;
        }
      }
      ++map_it;
    }

    // If the error code was more specific than the most specific entry found, we inform the user about that.
    if (subcode != error_code)
    {
      ss << "Error code could not be fully parsed. Full error code was " << hexToString(error_code) << std::endl;
    }

    return ss.str();
  }
  else
  {
    std::stringstream ss;
    ss << "Unknown emergency error code: " << hexToString(error_code);
    return ss.str();
  }
}

std::string EMCY::lookupErrorRegisterString ( const uint8_t error_code )
{
  std::stringstream ss;

  if (m_error_register_map.size() == 0)
  {
    ss << "No error register description set. Returning plain register code: "
       << hexToString(error_code) << std::endl;
  }
  else
  {
    for (size_t i = 0; i < (sizeof(error_code) * 8); ++i)
    {
      uint8_t bit = (0x01 << i) & error_code;
      std::map<uint8_t, std::string>::iterator map_it = m_error_register_map.find(bit);
      if (map_it != m_error_register_map.end())
      {
        ss << map_it->second << ", ";
      }
    }
  }

  if (ss.str().empty())
  {
    ss << "Unknown error register code: " << hexToString(error_code);
  }
  return ss.str();
}


std::string EMCY::lookupMSEFString() const
{
  return hexArrayToString(&m_msef[0], m_msef.size());
}

void EMCY::addEmergencyErrorMap(const std::string& filename, const std::string& block_identifier)
{
  std::map<uint32_t, std::string> new_entries = getErrorMapFromConfigFile(filename, block_identifier);
  for (std::map<uint32_t, std::string>::iterator it = new_entries.begin();
       it != new_entries.end();
       ++it)
  {
    uint16_t index = static_cast<uint16_t>(it->first);
    m_eec_map[index] = it->second;
  }
}

void EMCY::addErrorRegisterMap ( const std::string& filename, const std::string& block_identifier )
{
  std::map<uint32_t, std::string> new_entries = getErrorMapFromConfigFile(filename, block_identifier);
  for (std::map<uint32_t, std::string>::iterator it = new_entries.begin();
       it != new_entries.end();
       ++it)
  {
    uint8_t index = static_cast<uint8_t>(it->first);
    m_error_register_map[index] = it->second;
  }
}

void EMCY::printError (SDO& sdo, const uint8_t error_nr)
{
  std::vector<uint8_t> uploaded_data;
  sdo.upload(false, 0x1003, error_nr, uploaded_data);

  if (uploaded_data.size() != 4) // should be uint32
  {
    throw ProtocolException (0x1003, error_nr, "Uploaded data size does not match 4");
  }

  uint16_t eec = uploaded_data[0] + (uploaded_data[1] << 8);
  uint16_t additional_information = uploaded_data[2] + (uploaded_data[3] << 8);

  std::stringstream ss;
  ss << " Error " << static_cast<int>(error_nr) << ": "
     << lookupEECString(eec) << std::endl
     << "Additional information: " << hexToString(additional_information);
  LOGGING_ERROR_C(CanOpen, EMCY, ss.str() << endl);
}


void EMCY::printLastErrors(SDO& sdo)
{
  // first get the number of recorded errors
  uint8_t num_errors;
  sdo.upload(false, 0x1003, 0, num_errors);

  LOGGING_INFO_C (CanOpen, EMCY, num_errors << " errors in error history:" << endl);

  // get all present errors
  for (size_t i = 1; i <= num_errors; ++i)
  {
    printError (sdo, i);
  }

}

void EMCY::clearErrorHistory(SDO& sdo)
{
  // write 0 to 0x1003/0
  uint8_t value = 0;
  sdo.download(false, 0x1003, 0, value);
}




}}//end of NS
