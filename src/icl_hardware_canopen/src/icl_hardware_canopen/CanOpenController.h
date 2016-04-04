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
 * \date    2015-10-01
 *
 */
//----------------------------------------------------------------------

#ifndef CANOPENCONTROLLER_H
#define CANOPENCONTROLLER_H

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <icl_hardware_can/tCanDevice.h>

#include "helper.h"
#include "CanOpenReceiveThread.h"
#include "DS301Group.h"
#include "HeartBeatMonitor.h"

#ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
  // Schunk Diagnostics addon
  #include <icl_comm_websocket/WsBroadcaster.h>
#else
// Forward Deklaration of the WsBroadcaster
// This is not needed for normal driver operation
// but might be added later. To keep the interface the same
// a forward declaration becomes necessary
namespace icl_comm{
namespace websocket{
  class WsBroadcaster;
}}// NS end
#endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_

namespace icl_hardware{
namespace canopen_schunk{


/*!
 * \brief The CanOpenController class is the main entry point for any calls to the canOpen System
 *
 * The CanOpenController is the main entry point for any calls to the whole canOpen System.
 * It is used to set up the hardware (i.e. can device used), configure device groups and add individual devices
 * to these groups. It is also the entity providing access to the device groups, handling all the required
 * type conversions as to provide the most safe and convenient way to access the hardware.
 * Actual control commands (i.e. move a motor to a given position) are usually send directly to the corresponding
 * group as each group might present an individual interface.
 */
class CanOpenController : protected virtual boost::noncopyable
{
public:
  //! Shared pointer to a CanOpenController
  typedef boost::shared_ptr<CanOpenController> Ptr;
  //! Shared pointer to a const CanOpenController
  typedef boost::shared_ptr<const CanOpenController> ConstPtr;

  /*!
   * \brief CanOpenController constructor
   * \param can_device_identifier identifier string for the can device
   * \param baud_rate baud rate of the can device
   * \param resource_folder_location Folder location that contains ini files with error messages.
   * If none is given, it defaults to the environment variable CANOPEN_RESOURCE_PATH. If that is
   * not defined either, it searches in the \a resources subfolder where the program was launched.
   * If no error description is found at all, plain error codes will be displayed instead.
   */
  CanOpenController(const std::string can_device_identifier = "/dev/pcanusb0",
                    const uint32_t baud_rate = 500,
                    const std::string& resource_folder_location = "");

  ~CanOpenController ();

  /*!
   * \brief Initializes a node with a given ID or all nodes.
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  void initNodes(const int16_t node_id = -1);

  /*!
   * \brief Incoming can messages are processed in this function and forwarded to the addressed node
   *
   * \param msg Can message received by \a m_receive_thread
   */
  void processCanMsgCallback (const icl_hardware::can::tCanMessage& msg);

  /*!
   * \brief Reconnects the CAN device with the given configuration
   *
   * \param can_identifier Identifier string for the can device
   * \param baud_rate Baud rate of the can device
   */
  void reconnectCanDevice(const std::string& can_identifier, const uint32_t baud_rate);

  /*!
   * \brief Adds a new node group with a given identifier. The group's type is given
   * as template parameter
   *
   * \param identifier Name of the new group
   */
  template <typename GroupT>
  void addGroup(const std::string& identifier);

  /*!
   * \brief Returns a shared pointer to the group with a given index if possible.
   *
   * Note that this function will return a NULL pointer if either the given index
   * does not exist or if the group cannot be casted to the requested type. Error messages
   * are output in both cases. Therefore, check for NULL pointers after calling this
   * method.
   *
   * \param index The group's identifier string
   * \return boost::shared_ptr< GroupT > Shared pointer of type GroupT
   */
  template <typename GroupT>
  boost::shared_ptr<GroupT> getGroup (const std::string& index = "default");

  /*!
   * \brief Deletes a group with the given identifier. Also deletes all its nodes.
   *
   * \param identifier The group's identifier
   */
  void deleteGroup(const std::string& identifier);


  /*!
   * \brief Adds a new node to a group. If the group is not found (e.g. it was not created before),
   * nothing will be done.
   *
   * \param node_id ID of the new node
   * \param group_name The new node will belong to this group.
   */
  template <typename NodeT>
  void addNode(const uint8_t node_id, const std::string& group_name = "default");

  /*!
   * \brief Delete a node with a given CANOPEN-ID.
   *
   * \param node_id CANOPEN-ID of the node that should be deleted.
   */
  void deleteNode (const uint8_t node_id);

  /*!
   * \brief Get a handle to the current CAN device. This is basically for debugging
   * with a Dummy Can Device.
   *
   * \return icl_hardware::canopen_schunk::CanDevPtr Handle to the CAN device
   */
  CanDevPtr getCanDevice () const { return m_can_device; }

  /*!
   * \brief Downloads all the RPDOs, Sends a Sync and Uploads all the TPDOs.
   */
  void syncAll();

  /*!
   * \brief Returns a shared pointer handle to a node.
   *
   * \param node_id The node's CanOpen-ID
   * \throws NotFoundException when no node with the given ID can be found
   * \throws std::bad_cast when the node can't be casted into the requested template type.
   */
  template <typename NodeT>
  boost::shared_ptr<NodeT> getNode (const uint8_t node_id);

  /*!
   * \brief Calls stop() on all registered nodes.
   */
  void stopAll();

  /*!
   * \brief Returns a list containing all used node ids
   */
  std::vector<uint8_t> getNodeList ();

  /*!
   * \brief When using the ProfilePositionMode is used, in addition to setting the target motion
   * has to be triggered manually. Therefor, intermediate syncs are needed, which is why this is done
   * in the controller-.
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  void enablePPMotion(const int16_t node_id = -1);

private:
  //! Start can device and Message receiving
  //! throws a DeviceException if the device can't be initialized
  void init();

  //! Load resources like error-code lookup maps
  void getResources();

  //! get a handle for a node identified by its CANOPEN-ID
  DS301Node::Ptr getNodeById (const uint8_t node_id);

  //! Handle for the can receive thread
  CanOpenReceiveThreadPtr m_receive_thread;

  //! Handle for the can device
  CanDevPtr m_can_device;


  //@{
  /** Can device parameters */
  std::string m_can_device_name;
  int32_t m_can_device_flags;
  unsigned char m_can_device_acceptance_code;
  unsigned char m_can_device_acceptance_mask;
  uint32_t m_can_device_send_fifo_size;
  uint32_t m_can_device_receive_fifo_size;
  uint32_t m_can_device_baud_rate;
  //@}
  HeartBeatMonitor::Ptr m_heartbeat_monitor;

  //! The CAN message polling period duration in milliseconds
  uint32_t m_polling_period_ms;

  //! Map of all node groups, with string identifier
  std::map<std::string, DS301Group::Ptr> m_groups;
  //! Map of all nodes with id identifier
  std::map<uint8_t, DS301Node::Ptr> m_nodes;

  //! This folder contains for example the error code definitions
  std::string m_resource_folder_location;

  //! Interface to send out diagnostics data. Only available if compiled with _IC_BUILDER_ICL_COMM_WEBSOCKET_
  boost::shared_ptr<icl_comm::websocket::WsBroadcaster> m_ws_broadcaster;
  size_t m_ws_broadcast_counter; //! Counts sync cycles
  size_t m_ws_broadcast_rate; //! Every 'rate' sync cycles send out robot state

};


}// End of NS
}

// include template implementations
#include "CanOpenController.hpp"

#endif // CANOPENCONTROLLER_H
