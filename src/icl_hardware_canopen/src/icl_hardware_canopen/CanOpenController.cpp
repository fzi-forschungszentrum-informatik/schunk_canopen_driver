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

#include "CanOpenController.h"
#include <icl_hardware_can/tCanDeviceDummy.h>
#include "Logging.h"
#include "ds301.h"
#include "sync.h"
#include "exceptions.h"

#include <cstdlib> // getenv
#include <boost/filesystem.hpp>

namespace icl_hardware {
namespace canopen_schunk {

CanOpenController::CanOpenController(const std::string can_device_identifier,
                                     const uint32_t baud_rate,
                                     const std::string& resource_folder_location
                                    )
  : m_can_device_name(can_device_identifier),
    m_can_device_flags(O_RDWR|O_NONBLOCK),
    m_can_device_acceptance_code(0xff),
    m_can_device_acceptance_mask(0xff),
    m_can_device_send_fifo_size(300),
    m_can_device_receive_fifo_size(2000),
    m_can_device_baud_rate(baud_rate),
    m_heartbeat_monitor(new HeartBeatMonitor()),
    m_polling_period_ms(1),
    m_resource_folder_location(resource_folder_location),
    m_ws_broadcast_counter(0),
    m_ws_broadcast_rate(5)
{
    init();
}

CanOpenController::~CanOpenController ()
{
  // Stop receiving thread
  if (m_receive_thread)
  {
    m_receive_thread->stop();
  }
}

void CanOpenController::initNodes (const int16_t node_id)
{
  for (std::map<uint8_t, DS301Node::Ptr>::iterator it = m_nodes.begin(); it != m_nodes.end(); ++it)
  {
    if (it->first == node_id || node_id < 0)
    {
      it->second->initNode();
    }
  }
}

void CanOpenController::processCanMsgCallback ( const can::tCanMessage& msg )
{
  if (msg.id == ds301::ID_NMT)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "NMT MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Message length: " << msg.dlc << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
  }
  else if (msg.id == ds301::ID_SYNC)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "SYNC MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
  }
  else if (msg.id >= ds301::ID_EMCY_MIN && msg.id <= ds301::ID_EMCY_MAX)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "EMCY MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
    DS301Node::Ptr node = getNodeById(msg.id - ds301::ID_EMCY_MIN+1);
    if (node)
    {
      try
      {
        node->m_emcy->update(msg);
      }
      catch (const std::exception& e)
      {
        LOGGING_ERROR (CanOpen, "Exception thrown in EMCY update function: " << e.what() << endl);
      }
    }
  }
  else if (msg.id == ds301::ID_TIME)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "TIME MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
  }
  else if (msg.id >= ds301::ID_TPDO1_MIN && msg.id <= ds301::ID_TPDO1_MAX)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "TPDO1 MESSAGE for node " << msg.id - ds301::ID_TPDO1_MIN+1 << " RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
    DS301Node::Ptr node = getNodeById(msg.id - ds301::ID_TPDO1_MIN+1);
    if (node)
    {
      try
      {
        node->m_tpdos.at(0)->update(msg);
      }
      catch (const std::exception& e)
      {
        LOGGING_ERROR (CanOpen, "Exception thrown in PDO update function: " << e.what() << endl);
      }
    }
  }
  else if (msg.id >= ds301::ID_RPDO1_MIN && msg.id <= ds301::ID_RPDO1_MAX)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "RPDO1 MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
  }
  else if (msg.id >= ds301::ID_TPDO2_MIN && msg.id <= ds301::ID_TPDO2_MAX)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "TPDO2 MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
    DS301Node::Ptr node = getNodeById(msg.id - ds301::ID_TPDO2_MIN+1);
    if (node)
    {
      try
      {
        node->m_tpdos.at(1)->update(msg);
      }
      catch (const std::exception& e)
      {
        LOGGING_ERROR (CanOpen, "Exception thrown in PDO update function: " << e.what() << endl);
      }
    }
  }
  else if (msg.id >= ds301::ID_RPDO2_MIN && msg.id <= ds301::ID_RPDO2_MAX)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "RPDO2 MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
  }
  else if (msg.id >= ds301::ID_TPDO3_MIN && msg.id <= ds301::ID_TPDO3_MAX)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "TPDO3 MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
    DS301Node::Ptr node = getNodeById(msg.id - ds301::ID_TPDO3_MIN+1);
    if (node)
    {
      try
      {
        node->m_tpdos.at(2)->update(msg);
      }
      catch (const std::exception& e)
      {
        LOGGING_ERROR (CanOpen, "Exception thrown in PDO update function: " << e.what() << endl);
      }
    }
  }
  else if (msg.id >= ds301::ID_RPDO3_MIN && msg.id <= ds301::ID_RPDO3_MAX)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "RPDO3 MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
  }
  else if (msg.id >= ds301::ID_TPDO4_MIN && msg.id <= ds301::ID_TPDO4_MAX)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "TPDO4 MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
    DS301Node::Ptr node = getNodeById(msg.id - ds301::ID_TPDO4_MIN+1);
    if (node)
    {
      try
      {
        node->m_tpdos.at(3)->update(msg);
      }
      catch (const std::exception& e)
      {
        LOGGING_ERROR (CanOpen, "Exception thrown in PDO update function: " << e.what() << endl);
      }
    }
  }
  else if (msg.id >= ds301::ID_RPDO4_MIN && msg.id <= ds301::ID_RPDO4_MAX)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "RPDO4 MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
  }
  else if (msg.id >= ds301::ID_TSDO_MIN && msg.id <= ds301::ID_TSDO_MAX)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "TSDO MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
    DS301Node::Ptr node = getNodeById(msg.id - ds301::ID_TSDO_MIN+1);
    if (node)
    {
      try
      {
        node->m_sdo.update(msg);
      }
      catch (const ResponseException& e)
      {
        LOGGING_ERROR (CanOpen, "Exception thrown in SDO update function: " << e.what() << endl);
      }
    }
  }
  else if (msg.id >= ds301::ID_RSDO_MIN && msg.id <= ds301::ID_RSDO_MAX)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "RSDO MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
    DS301Node::Ptr node = getNodeById(msg.id - ds301::ID_RSDO_MIN+1);
    if (node)
    {
      try
      {
        node->m_sdo.update(msg);
      }
      catch (const ResponseException& e)
      {
        LOGGING_ERROR (CanOpen, "Exception thrown in SDO update function: " << e.what() << endl);
      }
    }
  }
  else if (msg.id >= ds301::ID_NMT_ERROR_MIN && msg.id <= ds301::ID_NMT_ERROR_MAX)
  {
    LOGGING_DEBUG_C (CanOpen, CanOpenController, "NMT_ERROR MESSAGE RECEIVED" << endl);
    LOGGING_TRACE_C (CanOpen, CanOpenController,
                     "Message id: " << msg.id << endl
                     << "Data: " << hexArrayToString(msg.data, msg.dlc) << endl);
    uint8_t node_id = msg.id - ds301::ID_NMT_ERROR_MIN+1;
    DS301Node::Ptr node = getNodeById(node_id);

    if (node)
    {
      try
      {
        node->m_nmt.update(msg);
      }
      catch (const ResponseException& e)
      {
        LOGGING_ERROR (CanOpen, "Exception thrown in NMT update function: " << e.what() << endl);
      }
      if (msg.data[0] != 0x0)
      {
        m_heartbeat_monitor->addHeartbeat(node_id);
      }
    }
    else if ( msg.dlc == 1 && msg.data[0] == 0x0)
    {
      // TODO (optional): If byte 0: add new node (autodiscovery)
      LOGGING_INFO_C (CanOpen, CanOpenController, "NMT bootup of node " << msg.id - ds301::ID_NMT_ERROR_MIN+1 << endl);
    }
  }
}

void CanOpenController::reconnectCanDevice(const std::string& can_identifier, const uint32_t baud_rate)
{
  if (baud_rate != 0)
  {
    m_can_device_baud_rate = baud_rate;
  }
  m_can_device_name = can_identifier;

  if (m_receive_thread)
  {
    m_receive_thread->stop();
  }

  try
  {
    init();
  }
  catch (const DeviceException& e)
  {
    LOGGING_ERROR_C (CanOpen, CanOpenController, "Initializing device failed. Reason: " << e.what());
  }
}

void CanOpenController::init()
{
  #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
  m_ws_broadcaster = boost::shared_ptr<icl_comm::websocket::WsBroadcaster>(new icl_comm::websocket::WsBroadcaster(icl_comm::websocket::WsBroadcaster::eRT_LWA4P));
  #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_


  getResources();

  if (m_can_device_name == "Dummy")
  {
    m_can_device.reset(new icl_hardware::can::tCanDeviceDummy( m_can_device_name.c_str(),
                                                            m_can_device_flags,
                                                            m_can_device_acceptance_code,
                                                            m_can_device_acceptance_mask,
                                                            m_can_device_baud_rate,
                                                            m_can_device_send_fifo_size,
                                                            m_can_device_receive_fifo_size
                                                          ));
  }
  else if (m_can_device_name == "auto")
  {
    const boost::array<const std::string, 4> names = {"/dev/pcanusb0","/dev/pcanusb1","/dev/pcanusb2","/dev/pcanusb3"};
    bool can_found = false;
    LOGGING_INFO (CanOpen, "CAN Device was set to auto. " << endl);

    for (size_t num = 0; num < names.size() ; ++num)
    {
        m_can_device_name = names[num];
        LOGGING_INFO(CanOpen, "Trying CAN device: " << m_can_device_name << "... " << endl);
        try
        {
            m_can_device.reset(icl_hardware::can::tCanDevice::Create( m_can_device_name.c_str(),
                                                                      m_can_device_flags,
                                                                      m_can_device_acceptance_code,
                                                                      m_can_device_acceptance_mask,
                                                                      m_can_device_baud_rate,
                                                                      m_can_device_send_fifo_size,
                                                                      m_can_device_receive_fifo_size
                                                                    ));

            if (m_can_device)
            {
              if (!m_can_device->IsInitialized())
              {
                std::stringstream ss;
                continue;
              }
            }
            can_found = true;

            break;

        }
        catch (DeviceException const& e)
        {
            LOGGING_INFO(CanOpen, e.what() << endl);
            can_found = false;
        }
    }
    if (!can_found)
    {
        LOGGING_ERROR(CanOpen, " CAN DEVICE COULD NOT BE OPENED. \n >> Giving up.");
        exit(-123);
        return;
    }
  }
  else
  {
    m_can_device.reset(icl_hardware::can::tCanDevice::Create( m_can_device_name.c_str(),
                                                            m_can_device_flags,
                                                            m_can_device_acceptance_code,
                                                            m_can_device_acceptance_mask,
                                                            m_can_device_baud_rate,
                                                            m_can_device_send_fifo_size,
                                                            m_can_device_receive_fifo_size
                                                          ));
  }

  if (m_can_device)
  {
    if (!m_can_device->IsInitialized())
    {
      std::stringstream ss;
      ss << "FATAL: COULD NOT INITIALIZE CAN DEVICE in " << m_can_device_name;

      throw DeviceException(ss.str());
    }
  }
  else
  {
    std::stringstream ss;
    ss << "FATAL: COULD NOT GET VALID CAN DEVICE in " << m_can_device_name;

    throw DeviceException(ss.str());
  }

  m_receive_thread.reset(new CanOpenReceiveThread(icl_core::TimeSpan::createFromMSec(m_polling_period_ms),
                                                  m_can_device,
                                                  boost::bind(&CanOpenController::processCanMsgCallback, this, _1)
                                                 ));

  if (!m_receive_thread)
  {
    throw DeviceException("FATAL: Could not start listener thread for CAN bus.");
  }

  // add default DS402Group
  addGroup<DS402Group>("default");
  m_heartbeat_monitor->registerErrorCallback(boost::bind(&CanOpenController::stopAll, this));
}

void CanOpenController::deleteGroup(const std::string& identifier)
{
  std::string sanitized_identifier = sanitizeString(identifier);

  std::map<std::string, DS301Group::Ptr>::iterator group_it;
  group_it = m_groups.find(sanitized_identifier);
  if (group_it != m_groups.end())
  {
    std::vector<uint8_t> deleted_node_ids;
    group_it->second->deleteNodes(deleted_node_ids); // This will return the IDs of deleted nodes

    // Now also delete shared pointers in controller
    for (std::vector<uint8_t>::iterator it = deleted_node_ids.begin();
         it != deleted_node_ids.end();
         ++it)
    {
      // Find the node with given Id
      std::map<uint8_t, DS301Node::Ptr>::iterator node_it = m_nodes.find(*it);
      assert(node_it->second.use_count() == 1);
      m_nodes.erase(node_it);
    }

    m_groups.erase(group_it);
  }
  else
  {
    LOGGING_ERROR_C(CanOpen, CanOpenController, "No group with the given identifer " << sanitized_identifier
      << " exists. Not deleting anything." << endl);
  }
}

void CanOpenController::deleteNode(const uint8_t node_id)
{
  // Delete node in group
  for (std::map<std::string, DS301Group::Ptr>::iterator it = m_groups.begin();
       it != m_groups.end();
       ++it)
  {
    if (it->second->deleteNodeFromId(node_id))
    {
      // group for node_id was found and node was deleted.
      break;
    }
  }

  // Delete node in controller's list
  std::map<uint8_t, DS301Node::Ptr>::iterator node_it = m_nodes.find(node_id);
  assert(node_it->second.use_count() == 1);
  m_nodes.erase(node_it);
}

DS301Node::Ptr CanOpenController::getNodeById(const uint8_t node_id)
{
  std::map<uint8_t, DS301Node::Ptr>::iterator node_it = m_nodes.find(node_id);
  if (node_it == m_nodes.end())
  {
    LOGGING_ERROR_C(CanOpen, CanOpenController, "A node with the given ID " << node_id <<
                                                " does not exist. Therefore this CAN message will be ignored." << endl);
    // Return a NULL pointer
    return DS301Node::Ptr();
  }
  return node_it->second;
}

void CanOpenController::getResources()
{
  boost::filesystem::path resources_path(m_resource_folder_location);

  if (m_resource_folder_location == "")
  {
    char const* tmp = std::getenv("CANOPEN_RESOURCE_PATH");
    if (tmp == NULL)
    {
      LOGGING_WARNING_C(
          CanOpen,
          CanOpenController,
          "The environment variable 'CANOPEN_RESOURCE_PATH' could not be read. Using relative path 'resources/'" << endl);
      resources_path = boost::filesystem::path("resources");
    }
    else
    {
      resources_path = boost::filesystem::path(tmp);
    }
  }

  std::string sdo_errors_filename = (resources_path / boost::filesystem::path("SDO.ini")).string();
  SDO::addErrorMapFromFile( sdo_errors_filename );

  std::string emcy_emergency_errors_filename = (resources_path / boost::filesystem::path("EMCY.ini")).string();
  EMCY::addEmergencyErrorMap( emcy_emergency_errors_filename, "emergency_errors");
  EMCY::addErrorRegisterMap( emcy_emergency_errors_filename, "error_registers");

  emcy_emergency_errors_filename = (resources_path / boost::filesystem::path("EMCY_DS402.ini")).string();
  EMCY::addEmergencyErrorMap( emcy_emergency_errors_filename, "emergency_errors");
}

void CanOpenController::syncAll()
{
  for (std::map<uint8_t, DS301Node::Ptr>::iterator it = m_nodes.begin();
       it != m_nodes.end();
  ++it)
  {
    it->second->downloadPDOs();
  }

  sendSync (m_can_device);

  for (std::map<uint8_t, DS301Node::Ptr>::iterator it = m_nodes.begin();
       it != m_nodes.end();
  ++it)
  {
    it->second->uploadPDOs();
  }

  #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
  if(m_ws_broadcast_counter % m_ws_broadcast_rate == 0)
  {
    if (!m_ws_broadcaster->sendState())
    {
    //    LOGGING_WARNING_C(
    //        CanOpen,
    //        CanOpenController, "Can't send ws_broadcaster state - reconnect pending..." << endl);
    }
    m_ws_broadcast_counter = 0;
  }
  ++m_ws_broadcast_counter;
  #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_

}

void CanOpenController::stopAll()
{
  LOGGING_INFO (CanOpen, "Stop of all nodes requested!" << endl);
  for (std::map<uint8_t, DS301Node::Ptr>::iterator it = m_nodes.begin();
       it != m_nodes.end();
  ++it)
  {
    it->second->stopNode();
  }
}

std::vector< uint8_t > CanOpenController::getNodeList()
{
  std::vector <uint8_t> node_list;

  size_t i=0;
  for (std::map<uint8_t, DS301Node::Ptr>::iterator it = m_nodes.begin();
       it != m_nodes.end();
  ++it)
  {
    it->second->downloadPDOs();
    node_list.push_back(it->second->getNodeId());
    ++i;
  }

  return node_list;
}



void CanOpenController::enablePPMotion(const int16_t node_id)
{
  syncAll();
  usleep(5000); // sleep 5 ms

  LOGGING_TRACE_C (CanOpen, CanOpenController, "in function " << __FUNCTION__ << endl);

  for (std::map<std::string, DS301Group::Ptr>::iterator it = m_groups.begin();
       it != m_groups.end();
       ++it)
  {
    DS402Group::Ptr group_ds402 = boost::dynamic_pointer_cast<DS402Group>(it->second);
    if (group_ds402)
    {
      group_ds402->startPPMovement(node_id);
    }
  }

  syncAll();
  usleep(5000); // sleep 5 ms

  for (std::map<std::string, DS301Group::Ptr>::iterator it = m_groups.begin();
       it != m_groups.end();
       ++it)
  {
    DS402Group::Ptr group_ds402 = boost::dynamic_pointer_cast<DS402Group>(it->second);
    if (group_ds402)
    {
      group_ds402->acceptPPTargets(node_id);
    }
  }
  syncAll();
  usleep(5000); // sleep 5 ms

}

}}//end of NS
