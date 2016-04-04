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

#include "SDO.h"
#include "ds301.h"

#include "Logging.h"

#include <boost/thread/mutex.hpp>

namespace icl_hardware {
namespace canopen_schunk {

// static initializer of error_map
std::map <uint32_t, std::string> SDO::m_error_map;

SDO::SDO(const uint8_t& node_id,const CanDevPtr& can_device)
  : m_node_id(node_id),
    m_can_device(can_device),
    m_response_wait_time_ms(100),
    m_data_update_received(false)
{
}

void SDO::update(const CanMsg& msg)
{
  uint8_t node_id = msg.id - ds301::ID_TSDO_MIN + 1;

  if (node_id != m_node_id)
  {
    std::stringstream ss;
    ss << "SDO Update called with wrong canopen ID. Received ID: " << static_cast<int>(node_id) << " Node ID: " << static_cast<int>(m_node_id) << ". Update ignored.";
    LOGGING_ERROR_C(CanOpen,SDO, ss.str() << endl);
    return;
  }

  boost::mutex::scoped_lock(m_data_buffer_mutex);

  if (msg.dlc != 8)
  {
    std::stringstream ss;
    ss << "Unexpected length " << static_cast<int>(msg.dlc) << " of SDO response. Expected 8.";
    throw ResponseException(0,0,ss.str());
  }

  if (m_data_update_received)
  {
    std::stringstream ss;
    ss << "Data buffer contains unprocessed data which will be overwritten.";
    LOGGING_WARNING_C (CanOpen, SDO, ss.str() << endl);
  }

  m_data_buffer.clear();
  for (size_t i = 0; i < msg.dlc; ++i)
  {
    m_data_buffer.push_back(msg.data[i]);
  }

  m_data_update_received = true;
  m_data_buffer_updated_cond.notify_one();
}

bool SDO::download(const bool normal_transfer,
                   const uint16_t index,
                   const uint8_t subindex,
                   const std::vector< uint8_t >& usrdata)
{
  //TODO: support normal transfer as well
  size_t num_bytes = usrdata.size();
  if (num_bytes > 4 || normal_transfer)
  {
    std::string mode = "expedited";
    if (normal_transfer)
    {
      mode = "segmented";
    }
    std::stringstream ss;
    ss << "So far only expedited transfers with maximum 4 data bytes are supported. "
       << "However, " << mode << " transfer of " << num_bytes
       << " bytes was requested. Aborting download";
    throw ProtocolException (index, subindex, ss.str());
  }

  if (num_bytes == 0)
  {
    throw ProtocolException (index, subindex, "Empty data message passed to download function.");
  }

  CanMsg msg;

  msg.id = ds301::ID_RSDO_MIN + m_node_id - 1;
  msg.dlc = 8;
  msg.rtr = 0;
  switch (num_bytes)
  {
  case 1:
      msg.data[0] = SDO_SEG_REQ_INIT_DOWNLOAD_1BYTE;
      break;
  case 2:
      msg.data[0] = SDO_SEG_REQ_INIT_DOWNLOAD_2BYTE;
      break;
  case 3:
      msg.data[0] = SDO_SEG_REQ_INIT_DOWNLOAD_3BYTE;
      break;
  case 4:
      msg.data[0] = SDO_SEG_REQ_INIT_DOWNLOAD_4BYTE;
      break;
  default:
      msg.data[0] = SDO_SEG_REQ_INIT_DOWNLOAD_xBYTE;
      break;
  }

  msg.data[1] = index & 0xff;
  msg.data[2] = index >> 8;
  msg.data[3] = subindex;
  for (size_t i = 0; i < 4; ++i)
  {
    if (i < num_bytes)
    {
      msg.data[4 + i] = usrdata[i];
    }
    else
    {
      msg.data[4 + i] = 0;
    }
  }

  // Send download request over CAN bus
  m_can_device->Send(msg);

  // Lock the data buffer access
  boost::mutex::scoped_lock buffer_guard(m_data_buffer_mutex);

  if (!m_data_update_received)
  {
    // Wait some time for the resonse...
    if (!m_data_buffer_updated_cond.timed_wait(buffer_guard,
                                               boost::posix_time::milliseconds(m_response_wait_time_ms)))
    {
      throw (TimeoutException(index, subindex, "No response to SDO download request received!"));
    }
  }

  /* New data will be processed next. As we might return from the function at many points,
   * we enable data receiving here again, as it will be locked by the mutex
   * until the end of this function, anyway.
   */
  m_data_update_received = false;


  // sanitizing again
  if ( m_data_buffer.size() != 8 )
  {
    std::stringstream ss;
    ss << "Unexpected length " << m_data_buffer.size() << " of SDO response. Expected 8.";
    throw (ProtocolException(index, subindex, ss.str()));
  }
  if ( m_data_buffer[0] == SDO_SEG_ABORT_TRANSFER )
  {
    uint32_t error_code = m_data_buffer[4] + (m_data_buffer[5]<<8) + (m_data_buffer[6]<<16) + (m_data_buffer[7]<<24);
    std::stringstream ss;
    ss << "SDO transfer aborted: " << lookupErrorString(error_code);
    throw ProtocolException(index, subindex, ss.str());
  }
  else if ( m_data_buffer[0] != SDO_SEG_RES_INIT_DOWNLOAD )
  {
    std::stringstream ss;
    ss << "Invalid SDO response, got " << hexToString(m_data_buffer[0])
        << " expected "  << hexToString(SDO_SEG_RES_INIT_DOWNLOAD);
    throw ResponseException(index, subindex, ss.str());
  }

  uint32_t rdindex = m_data_buffer[1] + (m_data_buffer[2]<<8);
  uint8_t rdsubindex = m_data_buffer[3];
  if (rdindex != index || rdsubindex != subindex)
  {
    std::stringstream ss;
    ss << "Invalid index/subindex, expected " << hexToString(index) << "/" << hexToString(subindex)
        << ", got " << hexToString(rdindex) << "/" << hexToString(rdsubindex);
    throw ResponseException(index, subindex, ss.str());
  }

  // If we reached this point, everything is fine :)
  return true;
}


bool SDO::upload(const bool normal_transfer,
                 const uint16_t index,
                 const uint8_t subindex,
                 std::vector<uint8_t>& uploaded_data)
{
  //TODO: support normal transfer as well
  if (normal_transfer)
  {
    LOGGING_ERROR_C(CanOpen, SDO, "So far only expedited transfers with maximum 4 data bytes "
                                  << "are supported. "
                                  << "However, blocked transfer of was requested. Aborting upload" << endl);
    return false;
  }

  CanMsg msg;

  msg.id = ds301::ID_RSDO_MIN + m_node_id - 1;
  msg.dlc = 8;
  msg.rtr = 0;
  msg.data[0] = SDO_SEG_REQ_INIT_UPLOAD;
  msg.data[1] = index & 0xff;
  msg.data[2] = index >> 8;
  msg.data[3] = subindex;

  // Send upload request over CAN bus
  m_can_device->Send(msg);

  // Lock the data buffer access
  boost::mutex::scoped_lock buffer_guard(m_data_buffer_mutex);

  if (!m_data_update_received)
  {
    // Wait some time for the response...
    if (!m_data_buffer_updated_cond.timed_wait(buffer_guard,
                                               boost::posix_time::milliseconds(m_response_wait_time_ms)))
    {
      throw (TimeoutException(index, subindex, "No response to SDO upload request received!"));
    }
  }

  /* New data will be processed next. As we might return from the function at many points,
   * we enable data receiving here again, as it will be locked by the mutex
   * until the end of this function, anyway.
   */
  m_data_update_received = false;

  // sanitizing again
  if ( m_data_buffer.size() != 8 )
  {
    std::stringstream ss;
    ss << "Unexpected length " << m_data_buffer.size() << " of SDO response. Expected 8.";
    throw ProtocolException (index, subindex, ss.str());
  }
  if ( m_data_buffer[0] == SDO_SEG_ABORT_TRANSFER )
  {
    uint32_t error_code = m_data_buffer[4] + (m_data_buffer[5]<<8) + (m_data_buffer[6]<<16) + (m_data_buffer[7]<<24);
    std::stringstream ss;
    ss << "SDO transfer aborted: " << lookupErrorString(error_code);
    throw ProtocolException(index, subindex, ss.str());
  }

  uint32_t rdindex = m_data_buffer[1] + (m_data_buffer[2]<<8);
  uint8_t rdsubindex = m_data_buffer[3];
  if (rdindex != index || rdsubindex != subindex)
  {
    std::stringstream ss;
    ss << "Invalid index/subindex, expected " << hexToString(index) << "/" << hexToString(subindex)
        << ", got " << hexToString(rdindex) << "/" << hexToString(rdsubindex);
    throw ResponseException (index, subindex, ss.str());
  }

  uploaded_data.clear();
  uint8_t num_bytes = 0;

  switch (m_data_buffer[0])
  {
    case SDO_SEG_RES_INIT_UPLOAD_1BYTE:
    {
      num_bytes = 1;
      break;
    }
    case SDO_SEG_RES_INIT_UPLOAD_2BYTE:
    {
      num_bytes = 2;
      break;
    }
    case SDO_SEG_RES_INIT_UPLOAD_3BYTE:
    {
      num_bytes = 3;
      break;
    }
    case SDO_SEG_RES_INIT_UPLOAD_4BYTE:
    {
      num_bytes = 4;
      break;
    }
    default:
    {
      std::stringstream ss;
      ss << "Illegal SDO upload response received. Please note that so far only expedited "
         << " uploads with a data length of up to 4 bytes are supported.\n"
         << "Received response was " << hexArrayToString(&m_data_buffer[0], m_data_buffer.size());
      throw ResponseException (index, subindex, ss.str());
    }
  }

  for (size_t i = 0; i < num_bytes; ++i)
  {
    uploaded_data.push_back(m_data_buffer[4+i]);
  }

  return true;
}

std::string SDO::lookupErrorString(const uint32_t error_code)
{
  std::map<uint32_t, std::string>::iterator map_it = m_error_map.find(error_code);
  if (map_it != m_error_map.end())
  {
    return map_it->second;
  }
  else
  {
    std::stringstream ss;
    ss << "Unknown error code: " << hexToString(error_code);
    return ss.str();
  }
}


void SDO::addErrorMapFromFile(const std::string& filename)
{
  std::map<uint32_t, std::string> new_entries = getErrorMapFromConfigFile(filename);
  m_error_map.insert(new_entries.begin(), new_entries.end());

  LOGGING_DEBUG(CanOpen, "Added error codes from " << filename << endl);
}


}}// end of NS
