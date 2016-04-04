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
 * \date    2015-10-1
 *
 */
//----------------------------------------------------------------------

#ifndef NMT_H
#define NMT_H

#include <stdint.h>
#include "helper.h"
#include "ds301.h"
#include <map>

namespace icl_hardware {
namespace canopen_schunk {

/*!
 * \brief The NMT class provides access to NMT functions of the canOpen protocol and keeps the NMT state of canOpen nodes
 *
 * NMT is the protocol layer that handles the state of individual nodes. It can be used to set the controlled devices into
 * operational state, shut them down etc.  This class provides access to those functions, handles incoming NMT messages and keeps
 * track of the NMT State of the individual nodes.
 */
class NMT
{
public:

  //! Node ID of all nodes at once
  static const uint8_t NMT_ALL_NODES = 0x00;

  //! NMT Command specifies what the state machine shall do value = "cs" as specified in ds301 7.2.8.3.1.1
  enum eNMT_Command
  {
      NMT_STARTREMOTENODE     = 0x01, // #1
      NMT_STOPREMOTENODE      = 0x02, // #2
      NMT_ENTERPREOPERATIONAL = 0x80, // #128
      NMT_RESETNODE           = 0x81, // #129
      NMT_RESETCOMMUNICATION  = 0x82  // #139
  };

  //! The NMT state indicates the behavior of the communication of a device, everything else is device specific. Only state operational allows PDO communication. See DS301 7.3.2.2
  enum eNMT_State
  {
    NMTS_STOPPED          = 0x04,
    NMTS_PRE_OPERATIONAL  = 0x7F,
    NMTS_OPERATIONAL      = 0x05,
    NMTS_INITIALISATION   = 0x00    // Value given arbitrarily, not defined in protocol
  };

  //! The NMT Substate is only used during initialization of a device. Meaning the substates are only useful if we want to implement a slave ourselves
  enum eNMT_SubState
  {
    NMTSS_INITIALISING,
    NMTSS_RESET_APPLICATION,
    NMTSS_RESET_COMMUNICATION
  };

  /*!
   * \brief NMT Construct a new NMT object to manage the NMT state of a device
   * \param node_id Canopen ID of the node to manage
   * \param can_device Can device handle for message sending
   */
  NMT(const uint8_t& node_id,const CanDevPtr& can_device);

  /*!
   * \brief update Updates the NMT status with newly received data
   * \param msg can message that has been identified as NMT message
   *
   * This function updates the NMT State of nodes by parsing received NMT messages.
   *
   * NMT:
   * AN NMT message with the ID 0x00 is the NMT command message. This is only relevant in case we are a slave node.
   *
   * NMT ERROR/HEARTBEAT:
   * CAN-ID =1792 + Node ID = Boot Up Message from the devices when they change from initialization to pre operational (payload = 0)
   * This transition is automatic once the device has initialized all its values.
   * If the Heartbeat node guarding is used the device will also send the same message periodically indicating
   * the NMT state in the lower 7 bit of the 1 byte payload. Bit 7 is always 0. The value sent is given in as eNMT_State value.
   * In case the heartbeat time is configured, the heartbeat producer time starts immediately, in case it remains unchanged the
   * time starts after the bootup event.
   * The Heartbeat protocol is used if the heartbeat producer time is unequal 0.
   *
   * NMT ERROR/NODE GUARDING:
   * With Node Guarding protocol the master (we) poll the device information by sending a remote transmission request to the node
   * The node answers wit CAN-ID = 1792 + Node-ID messages as with the heartbeat. Bit 0..6 are the status as with the heartbeat. Bit 7 is a toggle bit that
   * needs to alternate.
   * Requests are sent in intervals given by the node guarding time. If no response is received within the node life time = node guarding time * lifetime factor
   * the master indicates a "node guarding event" and the device a "life guarding event".
   * If the received status does not match the expected one a "node guarding event" is to be triggered (meaning a fault occurred).
   */
  void update(const CanMsg& msg);

  /*!
   * \brief start Starts the device by setting the NMT state to operational
   */
  void start();

  /*!
   * \brief stop Stops the device by setting the NMT state to stopped
   */
  void stop();

  /*!
   * \brief preOperational switches the device back into pre-operational state where configuration can occur
   */
  void preOperational();

  /*!
   * \brief reset Resets the device and triggers a complete reboot
   */
  void reset();

  /*!
   * \brief resetCommunication Resets the communication of a device, setting communication values to their defaults
   */
  void resetCommunication();



private:

  /*!
   * \brief commandToString Returns a string corresponding to a command enum.
   * \note One of the few options to "stringify" values without initializer lists (c++98)
   * \param cmd An NMT command
   * \return A string representing the given command
   */
  const std::string nmtCommandToString(const eNMT_Command& cmd)
  {
    std::string ret;
    switch (cmd)
    {
      case NMT_STARTREMOTENODE: ret = "start remote node"; break;
      case NMT_STOPREMOTENODE: ret = "stop remote node"; break;
      case NMT_ENTERPREOPERATIONAL: ret = "enter pre-operational"; break;
      case NMT_RESETNODE: ret = "reset node"; break;
      case NMT_RESETCOMMUNICATION: ret = "reset communication"; break;
      default: ret = "undefined"; break;
    }
    return ret;
  }

  /*!
   * \brief nmtStateToString Returns a string corresponding to a given NMT state
   * \note One of the few options to "stringify" values without initializer lists (c++98)
   * \param state An NMT state
   * \return A string representing the given state
   */
  const std::string nmtStateToString(const eNMT_State& state)
  {
    std::string ret;
    switch (state) {
      case NMTS_STOPPED: ret = "stopped" ; break;
      case NMTS_PRE_OPERATIONAL: ret = "pre operational" ; break;
      case NMTS_OPERATIONAL: ret = "operational" ; break;
      case NMTS_INITIALISATION: ret = "initialisation" ; break;
      default:ret = "undefined"; break;
    }
    return ret;
  }

  /*!
   * \brief isValidNmtState Helper function to check if a received value is a valid NMT state
   * \param state Value to check if it is an NMT state
   * \return true if the value is a valid NMT state
   */
  bool isValidNmtState(const uint8_t& state)
  {
    return ((state == NMTS_STOPPED) ||  (state == NMTS_PRE_OPERATIONAL) || (state == NMTS_OPERATIONAL) || (state == NMTS_INITIALISATION));
  }

  /*!
   * \brief sendCommand Sends an actual NMT command to the device. This will proactively
   * change the state of the NMT object which is important for the error handling.
   * \param cmd Command to send via NMT protocol
   */
  void sendCommand(const eNMT_Command& cmd);





  //! can device handle for sending of NMT commands
  CanDevPtr m_can_device;

  //! We keep the node ID which this NMT object is corresponding to
  uint8_t m_node_id;

  //! Status of the NMT state machine
  eNMT_State m_state;



};


}}//end of NS

#endif // NMT_H
