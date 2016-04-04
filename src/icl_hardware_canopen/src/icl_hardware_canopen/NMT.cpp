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

#include "NMT.h"
#include "Logging.h"

namespace icl_hardware {
namespace canopen_schunk {

NMT::NMT(const uint8_t &node_id, const CanDevPtr &can_device):
  m_can_device(can_device),
  m_node_id(node_id),
  m_state(NMTS_INITIALISATION)  // All canopen devices start in this mode and send an NMT message once they switch to pre-operational
{
}

void NMT::update(const CanMsg &msg)
{
  uint8_t node_id = (msg.id - ds301::ID_NMT_ERROR_MIN)+1;

  if (node_id != m_node_id)
  {
    LOGGING_ERROR_C(CanOpen,NMT,"NMT Update called with wrong canopen ID. Received ID: " << node_id << " Node ID: " << m_node_id << ". Update ignored." << endl);
    return;
  }

  // NMT messages will ALWAYS be 1 byte long
  // Only exception: When we are the receiving end, someone could send commands to us. However the driver is not designed for that.
  if (msg.dlc != 1)
  {
    LOGGING_ERROR_C(CanOpen,NMT,"NMT Update called with illegal length message. Expected length: " << 1 << " got: " << msg.dlc << ". Update ignored." << endl);
    return;
  }

  uint8_t payload = msg.data[0];

  // The Bootup messages are always right
  if (payload == 0)
  {
    LOGGING_INFO_C(CanOpen,NMT,"NMT Bootup complete for node " << node_id  << endl);
    m_state = NMTS_PRE_OPERATIONAL;
  }
  else
  {
    // Here comes the handling of Heartbeat or nodeguard messages
    // Regardless of protocol we want to change our current state if it is not already the right one
    // We mask bit 1 as it is the toggle bit
    uint8_t state_to_check = (payload & 0x7F);
    if (isValidNmtState(state_to_check))
    {
      if (m_state != static_cast<eNMT_State>(state_to_check))
      {
        // HEARTBEAT OR NODEGUARDING FAIL!
        LOGGING_WARNING_C(CanOpen,NMT,"NMT Nodeguarding for node " << node_id << " detected a failure! State is supposed to be: " << nmtStateToString(m_state) << " but was detected as " << nmtStateToString(static_cast<eNMT_State>(state_to_check)) << ". State changed." << endl);
        m_state = static_cast<eNMT_State>(state_to_check);
      }
      else
      {
      // If the states match, everything is fine!
        LOGGING_TRACE_C(CanOpen,NMT,"NMT Nodeguarding for node " << node_id << " received. ALL IS WELL!");
      }
    }
    else
    {
      LOGGING_DEBUG_C(CanOpen,NMT,"NMT Nodeguarding for node " << node_id << " received illegal NMT state information. Ignoring message" << endl);
    }
  }

}

void NMT::start()
{
 sendCommand(NMT_STARTREMOTENODE);
}

void NMT::stop()
{
 sendCommand(NMT_STOPREMOTENODE);
}

void NMT::preOperational()
{
 sendCommand(NMT_ENTERPREOPERATIONAL);
}

void NMT::reset()
{
  sendCommand(NMT_RESETNODE);
}

void NMT::resetCommunication()
{
  sendCommand(NMT_RESETCOMMUNICATION);
}

void NMT::sendCommand(const NMT::eNMT_Command& cmd)
{
  // The transition 2 (initialization to Pre-Operational) is an automatic one and can not be forced
  // proactively set the internal state to the desired one.
  switch (cmd)
  {
    case  NMT_STARTREMOTENODE:  // Transition 3,6 or none
      m_state = NMTS_OPERATIONAL;
      break;
    case  NMT_STOPREMOTENODE:
      m_state = NMTS_STOPPED; // Transition 5,8 or none
      break;
    case  NMT_ENTERPREOPERATIONAL:
      m_state = NMTS_PRE_OPERATIONAL; //Transition 4,7 or none
      break;
    case  NMT_RESETNODE:
      m_state = NMTS_INITIALISATION;  // Transition 9,10,11 All values will be reset to turn on default, device reboots
      break;
    case  NMT_RESETCOMMUNICATION:
      m_state = NMTS_INITIALISATION;  // Transition 12,13,14 All communication values will be reset. Device has to finish bootup sequence again.
      break;
    default:
      LOGGING_ERROR_C(CanOpen,NMT,"Illegal NMT command " << cmd << " was sent to node with id " << m_node_id << " . Command ignored." << endl);
      return; // do not send anything
      break;
  }

  // Send the data straight forward
  // The NMT Commands are without response so no further actions have to be taken for the communication side.
  unsigned char msg[2];
  msg[0] = static_cast<unsigned char>(cmd);
  msg[1] = m_node_id;
  m_can_device->Send(CanMsg(ds301::ID_NMT,2,0,msg));
}

}}//end of NS
