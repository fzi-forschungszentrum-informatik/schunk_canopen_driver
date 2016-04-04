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

#include "DS402Node.h"

#include "exceptions.h"
#include "sync.h"



namespace icl_hardware {
namespace canopen_schunk {

DS402Node::DS402Node(const uint8_t node_id, const CanDevPtr& can_device, HeartBeatMonitor::Ptr heartbeat_monitor):
  DS301Node(node_id,can_device, heartbeat_monitor),
  m_interpolation_cycle_time_ms(20),
  m_max_number_of_state_transitions(10),
  m_homing_method(0),
  m_transmission_factor(1)
{
}

void DS402Node::setDefaultPDOMapping (const DS402Node::eDefaultPDOMapping mapping)
{
  PDO::MappingConfigurationList rpdo_mappings;
  PDO::MappingConfigurationList tpdo_mappings;
  switch (mapping)
  {
    case PDO_MAPPING_CONTROLWORD_STATUSWORD:
    {
      // Map control and status word to the first PDO (receive and transmit respectively)
      rpdo_mappings.push_back(PDO::MappingConfiguration(0x6040, 0, 16, "control_word"));
      tpdo_mappings.push_back(PDO::MappingConfiguration(0x6041, 0, 16, "status_word"));
      initPDOMappingSingle (rpdo_mappings, 0, PDO::SYNCHRONOUS_CYCLIC, DS301Node::RECEIVE_PDO);
      initPDOMappingSingle (tpdo_mappings, 0, PDO::SYNCHRONOUS_CYCLIC, DS301Node::TRANSMIT_PDO);
      break;
    }
    case PDO_MAPPING_INTERPOLATED_POSITION_MODE:
    {
      // Map control and status word to the first PDO (receive and transmit respectively)
      rpdo_mappings.push_back(PDO::MappingConfiguration(0x6040, 0, 16, "control_word"));
      rpdo_mappings.push_back(PDO::MappingConfiguration(0x60C1, 1, 32, "interpolation_buffer"));

      tpdo_mappings.push_back(PDO::MappingConfiguration(0x6041, 0, 16, "status_word"));
      tpdo_mappings.push_back(PDO::MappingConfiguration(0x6064, 0, 32, "measured_position"));

      // Schunk powerballs have already mapped default PDOs. So we use a dummy mapping
      initPDOMappingSingle (rpdo_mappings, 0, PDO::SYNCHRONOUS_CYCLIC, DS301Node::RECEIVE_PDO);
      initPDOMappingSingle (tpdo_mappings, 0, PDO::SYNCHRONOUS_CYCLIC, DS301Node::TRANSMIT_PDO);
      break;
    }
    case PDO_MAPPING_PROFILE_POSITION_MODE:
    {
      // Map control and status word to the first PDO (receive and transmit respectively)
      rpdo_mappings.push_back(PDO::MappingConfiguration(0x6040, 0, 16, "control_word"));
      rpdo_mappings.push_back(PDO::MappingConfiguration(0x607A, 0, 32, "target_position"));

      tpdo_mappings.push_back(PDO::MappingConfiguration(0x6041, 0, 16, "status_word"));
      tpdo_mappings.push_back(PDO::MappingConfiguration(0x6064, 0, 32, "measured_position"));

      // Schunk powerballs have already mapped default PDOs. So we use a dummy mapping
      initPDOMappingSingle (rpdo_mappings, 0, PDO::SYNCHRONOUS_CYCLIC, DS301Node::RECEIVE_PDO);
      initPDOMappingSingle (tpdo_mappings, 0, PDO::SYNCHRONOUS_CYCLIC, DS301Node::TRANSMIT_PDO);
      break;
    }
    default:
    {
      break;
    }
  }

}


bool DS402Node::setTarget(const float target)
{
  int64_t target_ticks = m_transmission_factor * target;
  bool success = false;
  switch (m_op_mode)
  {
    case ds402::MOO_PROFILE_POSITION_MODE:
    {
      success = setRPDOValue("target_position", static_cast<int32_t>(target_ticks));
      break;
    }
    case ds402::MOO_VELOCITY_MODE:
    {
      success = setRPDOValue("vl_target_velocity", static_cast<int16_t>(target_ticks)); //6042
      break;
    }
    case ds402::MOO_PROFILE_VELOCITY_MODE:
    {
      success = setRPDOValue("target_velocity", static_cast<int32_t>(target_ticks)); //60FF
      break;
    }
    case ds402::MOO_PROFILE_TORQUE_MODE:
    {
      success = setRPDOValue("target_torque", static_cast<int16_t>(target_ticks)); //6071
      break;
    }
    case ds402::MOO_HOMING_MODE:
    {
      LOGGING_ERROR_C (CanOpen, DS402Node, "Homing mode does not know a target value." << endl);
      success = false;
      break;
    }
    case ds402::MOO_INTERPOLATED_POSITION_MODE:
    {
      success = setRPDOValue("interpolation_buffer", static_cast<int32_t>(target_ticks));
      break;
    }
    case ds402::MOO_CYCLIC_SYNC_POSITION_MODE:
    {
      LOGGING_WARNING_C (CanOpen, DS402Node, "Target for cyclic sync position mode is not yet supported." << endl);
      success = false;
      break;
    }
    case ds402::MOO_CYCLIC_SYNC_VELOCITY_MODE:
    {
      LOGGING_WARNING_C (CanOpen, DS402Node, "Target for cyclic sync velocity mode is not yet supported." << endl);
      success = false;
      break;
    }
    case ds402::MOO_CYCLIC_SYNC_TORQUE_MODE:
    {
      LOGGING_WARNING_C (CanOpen, DS402Node, "Target for cyclic sync torque mode is not yet supported." << endl);
      success = false;
      break;
    }
    default:
    {
      LOGGING_ERROR_C (CanOpen, DS402Node, "No legal mode of operation is set. setTarget() is non-functional. " << endl);
      break;
    }
  }
  if (success)
  {
    LOGGING_DEBUG_C(CanOpen, DS402Node, "Set target " << target << " for node " << m_node_id << endl);

  }
  return success;
}

void DS402Node::startPPMovement()
{
  ds402::Controlword word;
  word.all = getRPDOValue<uint16_t> ("control_word");
  word.bit.operation_mode_specific_0 = 1;
  setRPDOValue ("control_word", word.all);
  LOGGING_DEBUG_C(CanOpen, DS402Node, "Triggered motion for node " << m_node_id << endl);

}

void DS402Node::acceptPPTargets ()
{
  ds402::Controlword word;
  word.all = getRPDOValue<uint16_t> ("control_word");
  word.bit.operation_mode_specific_0 = 0;
  setRPDOValue ("control_word", word.all);
  LOGGING_DEBUG_C(CanOpen, DS402Node, "Accepting new targets for node " << m_node_id << endl);

}

double DS402Node::getTargetFeedback()
{
  int32_t feedback;
  switch (m_op_mode)
  {
    case ds402::MOO_PROFILE_POSITION_MODE:
    {
      feedback = getTPDOValue<int32_t>("measured_position");
      break;
    }
    case ds402::MOO_VELOCITY_MODE:
    {
      // TODO: implement me
      LOGGING_ERROR (CanOpen, "GetTargetFeature is not yet implemented for velocity mode." << endl);
      return 0;
    }
    case ds402::MOO_PROFILE_VELOCITY_MODE:
    {
      // TODO: implement me
      LOGGING_ERROR (CanOpen, "GetTargetFeature is not yet implemented for profile velocity mode." << endl);
      return 0;
    }
    case ds402::MOO_PROFILE_TORQUE_MODE:
    {
      // TODO: implement me
      LOGGING_ERROR (CanOpen, "GetTargetFeature is not yet implemented for profile torque mode." << endl);
      return 0;
    }
    case ds402::MOO_HOMING_MODE:
    {
      // TODO: implement me
//       LOGGING_ERROR (CanOpen, "GetTargetFeature is not yet implemented for homing mode." << endl);
      return 0;
    }
    case ds402::MOO_INTERPOLATED_POSITION_MODE:
    {
      feedback = getTPDOValue<int32_t>("measured_position");
      break;
    }
    case ds402::MOO_CYCLIC_SYNC_POSITION_MODE:
    {
      // TODO: implement me
      LOGGING_ERROR (CanOpen, "GetTargetFeature is not yet implemented for cyclic sync position mode." << endl);
      return 0;
    }
    case ds402::MOO_CYCLIC_SYNC_VELOCITY_MODE:
    {
      // TODO: implement me
      LOGGING_ERROR (CanOpen, "GetTargetFeature is not yet implemented for cyclic sync velocity mode." << endl);
      return 0;
    }
    case ds402::MOO_CYCLIC_SYNC_TORQUE_MODE:
    {
      // TODO: implement me
      LOGGING_ERROR (CanOpen, "GetTargetFeature is not yet implemented for cyclic sync torque mode." << endl);
      return 0;
    }
    default:
    {
      LOGGING_ERROR_C (CanOpen, DS402Node, "No legal mode of operation is set. getTargetFeedback() is non-functional. " << endl);
      return 0;
    }
  }

  #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
  if (m_ws_broadcaster)
  {
    m_ws_broadcaster->robot->setJointPosition(feedback, m_node_id);
  }
  #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_


  return static_cast<double>(feedback) / m_transmission_factor;
}


void DS402Node::setNMTState(const NMT::eNMT_State state, const NMT::eNMT_SubState sub_state)
{
  // TODO: implement me!
}

void DS402Node::quickStop()
{
  if (m_current_ds402_state == ds402::STATE_OPERATION_ENABLE)
  {
    LOGGING_INFO (CanOpen, "Quick stop of node " << m_node_id << " requested!" << endl);
    std::string identifier = "control_word";
    ds402::Controlword word;
    try
    {
      word.all = getRPDOValue<uint16_t> (identifier);
      word.bit.enable_voltage = 1;
      word.bit.quick_stop = 0;
      word.bit.reset_fault = 0;
      word.bit.halt = 1;
      setRPDOValue (identifier, word.all);
    }
    catch (const PDOException& e)
    {
      LOGGING_WARNING_C (CanOpen, DS402Node, "Could not find a PDO for the control word. Using slower SDO calls." << endl);
      word.bit.enable_voltage = 1;
      word.bit.quick_stop = 0;
      word.bit.reset_fault = 0;
      word.bit.halt = 1;

      m_sdo.download(false, ds402::ID_CONTROL_WORD, 0x00, word.all);
    }
  }
  m_expected_ds402_state = ds402::STATE_QUICKSTOP_ACTIVE;
}

void DS402Node::stopNode()
{
  quickStop();
}


void DS402Node::initDS402State (const ds402::eState& requested_state)
{
  LOGGING_DEBUG (CanOpen, "State " << deviceStatusString(requested_state) << " requested for node "
  << static_cast<int>(m_node_id) << "" << endl);
  if (requested_state == ds402::STATE_START ||
      requested_state == ds402::STATE_NOT_READY_TO_SWITCH_ON ||
      requested_state == ds402::STATE_FAULT_REACTION_ACTIVE)
  {
    LOGGING_ERROR_C (CanOpen, DS402Node, "Illegal target state requested. Requested state was: " <<
                     ds402::deviceStatusString(requested_state) << ", however that state cannot be entered manually!" << endl);
    return;
  }

  // get current state
  ds402::Statusword status_word;
  status_word.all = getTPDOValue<uint16_t>("status_word");
  ds402::eState current_state = ds402::stateFromStatusword(status_word);


  if (current_state == ds402::STATE_FAULT)
  {
    bool reset_fault_successful = false;
    for (size_t i = 0; i < 5; ++i)
    {
      reset_fault_successful = resetFault();
      if (reset_fault_successful)
      {
        break;
      }
    }

    if (!reset_fault_successful)
    {
      LOGGING_INFO (CanOpen, "Unable to reset from fault state after 5 tries. " <<
                             "Trying to do a hard reset on the device." << endl);
      m_nmt.stop();
      usleep(100000);
      m_nmt.start();

      if (!resetFault())
      {
        std::stringstream ss;
        ss << "Could not perform fault reset for node "
           << static_cast<int>(m_node_id)
           << " after multiple tries. "
           << "Is the device hard-disabled?"
           << "\nOtherwise: Have you tried turning it off and on again?";
        throw (DeviceException(ss.str()));
      }
    }

    m_sdo.upload(false, 0x6041, 0x0, status_word.all );
    current_state = ds402::stateFromStatusword(status_word);
  }

  // To prevent a deadlock if something goes wrong with state changes, the number of changes is limited
  size_t num_transitions = 0;

  while (current_state != requested_state && num_transitions < m_max_number_of_state_transitions)
  {
    LOGGING_DEBUG (CanOpen, "Node " << static_cast<int>(m_node_id) << " currently in state " <<
     deviceStatusString(current_state) << endl);
    switch (current_state)
    {
      case ds402::STATE_SWITCH_ON_DISABLED:
      {
        if (requested_state > ds402::STATE_SWITCH_ON_DISABLED)
        {
          doDS402StateTransition(ds402::STATE_TRANS_SHUTDOWN); // transition 2
          m_expected_ds402_state = ds402::STATE_READY_TO_SWITCH_ON;
        }
        break;
      }
      case ds402::STATE_READY_TO_SWITCH_ON:
      {
        if (requested_state > ds402::STATE_READY_TO_SWITCH_ON)
        {
          doDS402StateTransition(ds402::STATE_TRANS_SWITCH_ON); // transition 3
          m_expected_ds402_state = ds402::STATE_SWITCHED_ON;
        }
        else
        {
          doDS402StateTransition(ds402::STATE_TRANS_QUICK_STOP); // transition 7
          m_expected_ds402_state = ds402::STATE_SWITCH_ON_DISABLED;
        }
        break;
      }
      case ds402::STATE_SWITCHED_ON:
      {
        if (requested_state > ds402::STATE_SWITCHED_ON)
        {
          doDS402StateTransition(ds402::STATE_TRANS_ENABLE_OPERATION); // transition 4
          m_expected_ds402_state = ds402::STATE_OPERATION_ENABLE;
        }
        else if (requested_state == ds402::STATE_SWITCH_ON_DISABLED)
        {
          doDS402StateTransition(ds402::STATE_TRANS_INITIALIZE); // transition 10
          m_expected_ds402_state = ds402::STATE_SWITCH_ON_DISABLED;
        }
        else
        {
          doDS402StateTransition(ds402::STATE_TRANS_SHUTDOWN); // transition 6
          m_expected_ds402_state = ds402::STATE_READY_TO_SWITCH_ON;
        }
        break;
      }
      case ds402::STATE_OPERATION_ENABLE:
      {
        if (requested_state > ds402::STATE_OPERATION_ENABLE)
        {
          doDS402StateTransition(ds402::STATE_TRANS_QUICK_STOP); // transition 11
          m_expected_ds402_state = ds402::STATE_QUICKSTOP_ACTIVE;
        }
        else if (requested_state == ds402::STATE_SWITCHED_ON)
        {
          doDS402StateTransition(ds402::STATE_TRANS_SWITCH_ON); // transition 5
          m_expected_ds402_state = ds402::STATE_SWITCHED_ON;
        }
        else if (requested_state == ds402::STATE_READY_TO_SWITCH_ON)
        {
          doDS402StateTransition(ds402::STATE_TRANS_SHUTDOWN); // transition 8
          m_expected_ds402_state = ds402::STATE_READY_TO_SWITCH_ON;
        }
        else if (requested_state == ds402::STATE_SWITCH_ON_DISABLED)
        {
          doDS402StateTransition(ds402::STATE_TRANS_INITIALIZE); // transition 9
          m_expected_ds402_state = ds402::STATE_SWITCH_ON_DISABLED;
        }
        break;
      }
      case ds402::STATE_QUICKSTOP_ACTIVE:
      {
        // Query quick_stop option code. If it is 5,6,7 or 8, then we can perform this transition
        // However, this is optional, so we have to check if it's possible.
        int16_t quick_stop_option_code = 0;
        try
        {
          m_sdo.download(false, 0x605a, 0, quick_stop_option_code);
        }
        catch (const ProtocolException& e)
        {
          LOGGING_WARNING (CanOpen, "Caught error while downloading quick-stop-option code. "
                                    << "Probably it is not implemented, as it is optional. "
                                    << "Will carry on assuming quick-stop-option-code = 0.\n"
                                    << "Error was: " << e.what() << endl
          );
        }
        if (quick_stop_option_code >=5 && quick_stop_option_code <=8 &&
            requested_state == ds402::STATE_OPERATION_ENABLE)
        {
          doDS402StateTransition(ds402::STATE_TRANS_ENABLE_OPERATION); // transition 16
          m_expected_ds402_state = ds402::STATE_OPERATION_ENABLE;
        }
        else
        {
          doDS402StateTransition(ds402::STATE_TRANS_INITIALIZE); // transition 12
          m_expected_ds402_state = ds402::STATE_SWITCH_ON_DISABLED;
        }
        break;
      }
      case ds402::STATE_FAULT:
      {
        LOGGING_ERROR_C (CanOpen, DS402Node, "Landed in FAULT state while performing transition to " <<
                                             "state " << ds402::deviceStatusString(requested_state) <<
                                             ". Starting all over again..." << endl);
        break;
      }
      case ds402::STATE_START:
      case ds402::STATE_FAULT_REACTION_ACTIVE:
      case ds402::STATE_NOT_READY_TO_SWITCH_ON:
      {
        // nothing to do here.
        break;
      }
      default:
      {
        // This should not happen
        LOGGING_ERROR_C (CanOpen, DS402Node, "Landed in unknown state while performing transition to " <<
                                             "state " << ds402::deviceStatusString(requested_state) <<
                                             ". Aborting state transition." << endl);
        break;
      }
    }

    // now update the current state via SDO
    m_sdo.upload(false, 0x6041, 0x0, status_word.all );
    current_state = ds402::stateFromStatusword(status_word);

    LOGGING_INFO (CanOpen, "Node " << static_cast<int>(m_node_id) << " reached state " <<
     deviceStatusString(current_state) << endl);
    m_current_ds402_state = current_state;
    setTPDOValue("status_word", status_word.all);

    num_transitions++;
  }
  m_expected_ds402_state = m_current_ds402_state;

}


void DS402Node::home()
{
  if (m_homing_method == 0)
  {
    LOGGING_WARNING (CanOpen, "Homing method for node " << static_cast<int>(m_node_id) <<
      " is not set. Aborting homing now." << endl);
    return;
  }

  LOGGING_INFO (CanOpen, "Starting homing for node " << static_cast<int>(m_node_id) << endl);

  setModeOfOperation(ds402::MOO_HOMING_MODE);

  initDS402State(ds402::STATE_OPERATION_ENABLE);

  ds402::Controlword controlword;
  controlword.all = getRPDOValue<uint16_t>("control_word");

  // home operation start
  controlword.bit.operation_mode_specific_0 = 1;
  controlword.bit.halt = 0;

  // activate the homing mode
  m_sdo.download(false, ds402::ID_CONTROL_WORD, 0x0, controlword.all);

  ds402::Statusword statusword;
  bool homing_attained = false;
  bool homing_error = false;
  while (!homing_attained)
  {
    m_sdo.upload(false, ds402::ID_STATUS_WORD, 0, statusword.all);
    homing_attained = statusword.bit.operation_mode_specific_0;
    homing_error = statusword.bit.operation_mode_specific_1;

    if (homing_error)
    {
      std::stringstream ss;
      ss << "Homing of node " << static_cast<int>(m_node_id) << " failed.";
      throw DeviceException(ss.str());
    }

    if (homing_attained)
    {
      // We're done homing
      LOGGING_INFO (CanOpen, "Done homing for node " << static_cast<int>(m_node_id) << endl);
      break;
    }

    usleep(100000); // wait 100 ms
  }
}

void DS402Node::printStatus()
{
  ds402::Statusword statusword;
  statusword.all = getTPDOValue<uint16_t> ("status_word");
  ds402::eState state = stateFromStatusword(statusword);
  std::stringstream ss;
  ss << "Device " << static_cast<int>(m_node_id) << " status: " <<
   binaryString(statusword.all) << "\n(state " << ds402::deviceStatusString(state)
   << ")" << std::endl;
  ss << "Fault: " << statusword.bit.fault << std::endl;
  ss << "Switched on: " << statusword.bit.switched_on << std::endl;
  ss << "Operation enabled: " << statusword.bit.operation_enabled << std::endl;
  ss << "Voltage enabled: " << statusword.bit.voltage_enabled << std::endl;
  ss << "Quick stop active: " << statusword.bit.quick_stop << std::endl;
  ss << "Target reached: " << statusword.bit.target_reached << std::endl;

  // add Bits 12 and 13 which are op_mode specific
  ss << operationModeSpecificStatus(statusword);

  LOGGING_INFO_C (CanOpen, DS402Node, ss.str() << endl);
}

std::string DS402Node::operationModeSpecificStatus (const ds402::Statusword& statusword)
{
  std::stringstream ss;
  switch (m_op_mode)
  {
    case ds402::MOO_PROFILE_POSITION_MODE:
    {
      ss << "Set-point acknowledge: " << statusword.bit.operation_mode_specific_0 << std::endl;
      ss << "Following error: " << statusword.bit.operation_mode_specific_1 << std::endl;
      break;
    }
    case ds402::MOO_VELOCITY_MODE:
    {
      break;
    }
    case ds402::MOO_PROFILE_VELOCITY_MODE:
    {
      ss << "Speed: " << statusword.bit.operation_mode_specific_0 << std::endl;
      ss << "Max slippage error: " << statusword.bit.operation_mode_specific_1 << std::endl;
      break;
    }
    case ds402::MOO_PROFILE_TORQUE_MODE:
    {
      break;
    }
    case ds402::MOO_HOMING_MODE:
    {
      ss << "Homing attained: " << statusword.bit.operation_mode_specific_0 << std::endl;
      ss << "Homing error: " << statusword.bit.operation_mode_specific_1 << std::endl;
      break;
    }
    case ds402::MOO_INTERPOLATED_POSITION_MODE:
    {
      ss << "Interpolated position mode active: " << statusword.bit.operation_mode_specific_0 << std::endl;
      break;
    }
    case ds402::MOO_CYCLIC_SYNC_POSITION_MODE:
    {
      break;
    }
    case ds402::MOO_CYCLIC_SYNC_VELOCITY_MODE:
    {
      break;
    }
    case ds402::MOO_CYCLIC_SYNC_TORQUE_MODE:
    {
      break;
    }
    default:
    {
      // do nothing
      break;
    }
  }

  return ss.str();
}

void DS402Node::querySupportedDeviceModes()
{
  m_sdo.upload(false, 0x6502, 0, m_supported_modes.all);
}

void DS402Node::initNode()
{
  setDefaultPDOMapping(PDO_MAPPING_PROFILE_POSITION_MODE);
  DS301Node::initNode();
  querySupportedDeviceModes();

  boost::function <void()> f = boost::bind(&DS402Node::onStatusWordUpdate, this);
  registerPDONotifyCallback("status_word", f);

  setModeOfOperation(ds402::MOO_PROFILE_POSITION_MODE);
}

bool DS402Node::isModeSupported (const ds402::eModeOfOperation op_mode)
{
  int8_t mode_int = op_mode;
  uint32_t bitmask_mode = 0x01 << (mode_int-1);

  if ((m_supported_modes.all & bitmask_mode) != bitmask_mode)
  {
    return false;
  }

  return true;
}

void DS402Node::printSupportedModesOfOperation()
{
  std::stringstream ss;
  ss << "Modes of operation supported by device " << static_cast<int>(m_node_id) << std::endl;
  if (m_supported_modes.bit.profile_position_mode)
  {
    ss << "Profile position mode" << std::endl;
  }
  if (m_supported_modes.bit.velocity_mode)
  {
    ss << "Velocity mode" << std::endl;
  }
  if (m_supported_modes.bit.profile_velocity_mode)
  {
    ss << "Profile velocity mode" << std::endl;
  }
  if (m_supported_modes.bit.profile_torque_mode)
  {
    ss << "Profile torque mode" << std::endl;
  }
  if (m_supported_modes.bit.homing_mode)
  {
    ss << "Homing mode" << std::endl;
  }
  if (m_supported_modes.bit.interpolated_position_mode)
  {
    ss << "Interpolated position mode" << std::endl;
  }
  if (m_supported_modes.bit.cyclic_sync_position_mode)
  {
    ss << "Cyclic sync position mode" << std::endl;
  }
  if (m_supported_modes.bit.cyclic_sync_velocity_mode)
  {
    ss << "Cyclic sync velocity mode" << std::endl;
  }
  if (m_supported_modes.bit.cyclic_sync_torque_mode)
  {
    ss << "Cyclic sync torque mode" << std::endl;
  }

  LOGGING_INFO (CanOpen, ss.str() << endl);
}


bool DS402Node::setModeOfOperation (const ds402::eModeOfOperation op_mode)
{
  // Changes of opMode are always possible. So make sure, the motor is standing while changing the opMode.
  if (m_current_ds402_state == ds402::STATE_OPERATION_ENABLE)
  {
    closeBrakes();
  }

  if (op_mode != ds402::MOO_HOMING_MODE &&
      op_mode != ds402::MOO_INTERPOLATED_POSITION_MODE &&
      op_mode != ds402::MOO_PROFILE_POSITION_MODE
     )
  {
    LOGGING_ERROR_C (CanOpen, DS402Node, "Requested to switch to mode " <<
    operationModeString(op_mode) << " for node " <<
    static_cast<int>(m_node_id) << ", which is currently not supported." << endl);
    return false;
  }

  if (!isModeSupported(op_mode))
  {
    LOGGING_ERROR_C (CanOpen, DS402Node, "The requested mode: " <<
      operationModeString(op_mode) <<
      " is not supported by the device " <<
      static_cast<int>(m_node_id) << "." << endl);
    return false;
  }

  if (op_mode == ds402::MOO_INTERPOLATED_POSITION_MODE)
  {
    configureInterpolationCycleTime();
    configureInterpolationData (0, 0, 4);
  }

  bool success = true;

  int8_t mode_int = op_mode;
  try
  {
    success = m_sdo.download(false, 0x6060, 0, mode_int);
  }
  catch (const std::exception& e)
  {
    LOGGING_ERROR_C (CanOpen, DS402Node, e.what() << endl);
  }
  if (success)
  {
    m_op_mode = op_mode;
    LOGGING_INFO (CanOpen, "Initialized mode " << ds402::operationModeString(op_mode) <<
     " for node " << m_node_id << endl);
  }

  return success;
}

void DS402Node::configureInterpolationCycleTime (const uint8_t interpolation_cycle_time_ms)
{
  if (interpolation_cycle_time_ms != 0)
  {
    m_interpolation_cycle_time_ms = interpolation_cycle_time_ms;
  }
  m_sdo.download(false, 0x60c2, 0x01, m_interpolation_cycle_time_ms);
  int8_t magnitude = -3; // milliseconds
  m_sdo.download(false, 0x60c2, 0x02, magnitude);
}

void DS402Node::setupProfilePositionMode (const ds402::ProfilePositionModeConfiguration& config)
{
  configureProfileAcceleration(config.profile_acceleration * m_transmission_factor);


  // If only an acceleration parameter is given, it will be used for deceleration, as well.
  float deceleration = config.profile_deceleration;
  if (config.profile_deceleration == 0)
  {
    deceleration = config.profile_acceleration;
  }


  // Some devices don't support setting deceleration parameter. This catches that.
  try
  {
    configureProfileDeceleration(deceleration * m_transmission_factor);
  }
  catch (const ProtocolException& e)
  {
    LOGGING_DEBUG (CanOpen, "Failed to set the profile deceleration for node " << m_node_id << ". " <<
                           " Some devices do not support setting this, but use the acceleration value for deceleration, as well. So you might want to ignore this message. " <<
                           "The error was: " << e.what() << endl
    );
  }

  // Some devices don't support setting it and 0 is the default
  if (config.motion_profile_type != 0)
  {
    configureMotionProfileType(config.motion_profile_type * m_transmission_factor);
  }
  configureProfileVelocity(config.profile_velocity * m_transmission_factor);

  m_ppm_config = config;
}

void DS402Node::setupHomingMode (const ds402::HomingModeConfiguration& config)
{
  configureHomingSpeeds (config.homing_speed_low, config.homing_speed_high);
  configureHomingMethod (config.homing_method);
}

void DS402Node::setupProfileVelocityMode (const ds402::ProfileVelocityModeConfiguration& config)
{
  configureSensorSelectionCode (config.sensor_selection_code);
}

void DS402Node::setupProfileTorqueMode (const ds402::ProfileTorqueModeConfiguration& config)
{
  configureTorqueSlope (config.torque_slope);
  configureTorqueProfileType (config.torque_profile_type);
}

bool DS402Node::resetFault ()
{
  ds402::Statusword status_word;
  status_word.all = getTPDOValue<uint16_t>("status_word");
  ds402::eState current_state = ds402::stateFromStatusword(status_word);

  if (current_state != ds402::STATE_FAULT)
  {
    LOGGING_INFO_C (CanOpen, DS402Node, "Requested resetFault action, but device is currently " <<
    "not in state FAULT. Instead it is in state " << deviceStatusString (current_state) <<
    ". Not doing anything here." << endl);
    return true;
  }

  // clear the error register
  m_emcy->clearErrorHistory(m_sdo);

  // enter SWITCH_ON_DISABLED state
  doDS402StateTransition(ds402::STATE_TRANS_FAULT_RESET); // transition 15

  usleep (100000); // wait 100ms
  status_word.all = getTPDOValue<uint16_t>("status_word");
  current_state = ds402::stateFromStatusword(status_word);

  if (current_state != ds402::STATE_SWITCH_ON_DISABLED)
  {
    LOGGING_ERROR_C (CanOpen, DS402Node, "Could not perform fault reset for node " <<
      m_node_id << ". Possibly the reason for entering the fault state still exists." <<
      endl
    );
    return false;
  }
  return true;
}


void DS402Node::configureMaxAcceleration (const uint32_t acceleration)
{
  m_sdo.download(false, 0x60c5, 0, acceleration);

  LOGGING_INFO_C (CanOpen, DS402Node, "Maximum acceleration for node " << m_node_id << " written." << endl);
}

void DS402Node::configureMaxDeceleration (const uint32_t deceleration)
{
  m_sdo.download(false, 0x60c6, 0, deceleration);
  LOGGING_INFO_C (CanOpen, DS402Node, "Maximum deceleration for node " << m_node_id << " written." << endl);
}

void DS402Node::configureProfileVelocity (const uint32_t velocity)
{
 m_sdo.download(false, 0x6081, 0, velocity);
 LOGGING_INFO_C (CanOpen, DS402Node, "Profile velocity for node " << m_node_id << " written with value " <<
                                        velocity << "." << endl);
}


void DS402Node::configureProfileAcceleration (const uint32_t acceleration)
{
  m_sdo.download(false, 0x6083, 0, acceleration);
  LOGGING_INFO_C (CanOpen, DS402Node, "Profile acceleration for node " << m_node_id << " written with value " <<
                                        acceleration << "." << endl);
}

void DS402Node::configureProfileDeceleration (const uint32_t deceleration)
{
  m_sdo.download(false, 0x6084, 0, deceleration);
}

void DS402Node::configureQuickStopDeceleration (const uint32_t deceleration)
{
  m_sdo.download(false, 0x6085, 0, deceleration);
  LOGGING_INFO_C (CanOpen, DS402Node, "Quick Stop deceleration for node " << m_node_id << " written with value " <<
                                        deceleration << "." << endl);
}

void DS402Node::configureMotionProfileType (const int16_t motion_type)
{
  m_sdo.download(false, 0x6086, 0, motion_type);
  LOGGING_INFO_C (CanOpen, DS402Node, "Motion profile type for node " << m_node_id << " written with value " <<
                                        motion_type << "." << endl);
}


void DS402Node::configureHomingSpeeds (const uint32_t low_speed, const uint32_t high_speed)
{
  m_sdo.download(false, 0x6099, 1, high_speed);
  m_sdo.download(false, 0x6099, 2, low_speed);

  LOGGING_INFO_C (CanOpen, DS402Node, "Homing speeds for node " << m_node_id << " written." << endl);

}

void DS402Node::configureHomingAcceleration (const uint32_t acceleration)
{
  m_sdo.download(false, 0x609A, 0, acceleration);
  LOGGING_INFO_C (CanOpen, DS402Node, "Homing acceleration for node " << m_node_id << " written." << endl);
}

void DS402Node::configureHomingMethod (const int8_t homing_method)
{
  m_sdo.download(false, 0x6098, 0, homing_method);
  LOGGING_INFO_C (CanOpen, DS402Node, "Homing method for node " << m_node_id << " written." << endl);
  m_homing_method = homing_method;
}

void DS402Node::configureSensorSelectionCode (const int16_t sensor_selection_code)
{
  m_sdo.download(false, 0x606a, 0, sensor_selection_code);
  LOGGING_INFO_C (CanOpen, DS402Node, "Sensor selection code for node " << m_node_id << " written." << endl);
}

void DS402Node::configureTorqueProfileType (const int16_t torque_profile_type)
{
  m_sdo.download(false, 0x6088, 0, torque_profile_type);
  LOGGING_INFO_C (CanOpen, DS402Node, "Torque profile type for node " << m_node_id << " written." << endl);
}

void DS402Node::configureTorqueSlope (const uint32_t torque_slope)
{
  m_sdo.download(false, 0x6087, 0, torque_slope);
  LOGGING_INFO_C (CanOpen, DS402Node, "Torque slope for node " << m_node_id << " written." << endl);
}

void DS402Node::configureInterpolationData (const uint8_t buffer_organization,
                                            const int16_t interpolation_type,
                                            const uint8_t size_of_data_record)
{
  uint8_t data8 = 0;
  // Reset the buffer
  m_sdo.download(false, 0x60c4, 6, data8);

  // Enable the buffer again
  data8 = 1;
  m_sdo.download(false, 0x60c4, 6, data8);

  // set linear interpolation
  int16_t data16 = 0;
  m_sdo.download(false, 0x60c0, 0, data16);

  m_sdo.download(false, 0x60c4, 3, buffer_organization);
  m_sdo.download(false, 0x60c4, 5, size_of_data_record);

  LOGGING_DEBUG_C (CanOpen, DS402Node, "Interpolation data for node " << m_node_id << " written." << endl);
}

void DS402Node::doDS402StateTransition (const ds402::eStateTransission transition)
{
  LOGGING_DEBUG_C (CanOpen, DS402Node, "Requested DS402 state transition " << transition << endl);
  // create according control word and send it via SDO, update it in the PDO cache
  std::string identifier = "control_word";
  ds402::Controlword word;
  word.all = getRPDOValue<uint16_t> (identifier);

  switch (transition)
  {
    case ds402::STATE_TRANS_INITIALIZE: // 1, 7, 9, 10, 12
    {
      word.bit.reset_fault = 0;
      word.bit.enable_voltage = 0;
      word.bit.operation_mode_specific_0 = 0;
      break;
    }
    case ds402::STATE_TRANS_SHUTDOWN: // 2, 6, 8
    {
      word.bit.reset_fault = 0;
      word.bit.quick_stop = 1;
      word.bit.enable_voltage = 1;
      word.bit.switch_on = 0;
      word.bit.operation_mode_specific_0 = 0;
      break;
    }
    case ds402::STATE_TRANS_SWITCH_ON: // 3, 5
    {
      word.bit.reset_fault = 0;
      word.bit.enable_operation = 0;
      word.bit.quick_stop = 1;
      word.bit.enable_voltage = 1;
      word.bit.switch_on = 1;
      word.bit.halt = 1;
      break;
    }
    case ds402::STATE_TRANS_ENABLE_OPERATION: // 4, 16
    {
      word.bit.reset_fault = 0;
      word.bit.enable_operation = 1;
      word.bit.quick_stop = 1;
      word.bit.enable_voltage = 1;
      word.bit.switch_on = 1;
      word.bit.halt = 0;
      break;
    }
    case ds402::STATE_TRANS_QUICK_STOP: // 11
    {
      word.bit.reset_fault = 0;
      word.bit.quick_stop = 0;
      word.bit.enable_voltage = 1;
      word.bit.halt = 1;
      break;
    }
    case ds402::STATE_TRANS_FAULT_RESET: // 15
    {
      word.bit.reset_fault = 1;
      break;
    }
    default:
    {
      std::stringstream ss;
      ss << "Illegal DS402 state transition requested: " << transition;
      throw ProtocolException (ds402::ID_CONTROL_WORD, 0x00, ss.str());
    }
  }

  // now send the controlword to the device
  m_sdo.download(false, ds402::ID_CONTROL_WORD, 0x0, word.all);
  LOGGING_DEBUG (CanOpen, "Sent controlword " << binaryString(word.all) << endl);

  // copy it to the pdo cache
  setRPDOValue(identifier, word.all);
}

void DS402Node::enableNode (const ds402::eModeOfOperation operation_mode)
{
  m_nmt.start();
  if (m_current_ds402_state == ds402::STATE_FAULT)
  {
    resetFault();
  }

  if (operation_mode != ds402::MOO_RESERVED_0)
  {
    setModeOfOperation(operation_mode);
  }
  initDS402State(ds402::STATE_OPERATION_ENABLE);
  // Set current position to target position to prevent jumps
  double feedback = getTargetFeedback();
  setTarget(feedback);

  openBrakes();
}

void DS402Node::disableNode()
{
  if (m_current_ds402_state == ds402::STATE_OPERATION_ENABLE)
  {
    closeBrakes();
  }

  initDS402State(ds402::STATE_SWITCHED_ON);
}

void DS402Node::openBrakes()
{
  if (m_current_ds402_state == ds402::STATE_OPERATION_ENABLE)
  {
    ds402::Controlword controlword;
    controlword.all = getRPDOValue<uint16_t>("control_word");


    // If the requested mode is InterpolatedPositionMode, we enable interpolation here
    if (m_op_mode == ds402::MOO_INTERPOLATED_POSITION_MODE)
    {
      controlword.bit.operation_mode_specific_0 = 1;
      controlword.bit.halt = 0;
    }
    else if (m_op_mode == ds402::MOO_PROFILE_POSITION_MODE)
    {
      controlword.bit.operation_mode_specific_0 = 0; // Bit 4

      controlword.bit.operation_mode_specific_1 = !(m_ppm_config.change_set_immediately); // Bit 5: move to set point immediately
      controlword.bit.operation_mode_specific_2 = m_ppm_config.use_relative_targets; // Bit 6
      controlword.bit.reserved_0                = m_ppm_config.use_blending; // Bit 9: use blending
      controlword.bit.halt                     = 0;
    }
    else
    {
      controlword.bit.operation_mode_specific_0 = 0;
      controlword.bit.halt = 0;
    }
    setRPDOValue("control_word", controlword.all);
  }
  else
  {
    LOGGING_ERROR (CanOpen, "OpenBrakes called while not in OPERATION_ENABLE state. Will do nothing" << endl);
  }
}


void DS402Node::closeBrakes()
{
  if (m_current_ds402_state == ds402::STATE_OPERATION_ENABLE)
  {
    ds402::Controlword controlword;
    m_sdo.upload(false, 0x6040, 0, controlword.all);
    // If the requested mode is InterpolatedPositionMode, we disable interpolation here
    if (m_op_mode == ds402::MOO_INTERPOLATED_POSITION_MODE)
    {
      controlword.bit.operation_mode_specific_0 = 0;
    }
    controlword.bit.halt = 1;
    setRPDOValue("control_word", controlword.all);
  }
  else
  {
    LOGGING_ERROR (CanOpen, "CloseBrakes called while not in OPERATION_ENABLE state. Will do nothing" << endl);
  }
}

void DS402Node::onStatusWordUpdate()
{
  ds402::Statusword statusword;
  statusword.all = getTPDOValue<uint16_t>("status_word");

  // check if DS402 status  matches the one from the device

  ds402::eState device_state =  ds402::stateFromStatusword(statusword);
  if (m_current_ds402_state != device_state)
  {
    if (m_expected_ds402_state != device_state)
    {
      LOGGING_WARNING (CanOpen, "The device " << m_node_id <<
                                " has switched to state " <<
                                ds402::deviceStatusString(device_state) <<
                                " without host request. " <<
                                "The controller will adapt the device's status." << endl
      );
    }
    m_current_ds402_state = device_state;
  }
}

bool DS402Node::isTargetReached()
{
  ds402::Statusword statusword;
  statusword.all = getTPDOValue<uint16_t>("status_word");

  return statusword.bit.target_reached;
}

ds402::Statusword DS402Node::getStatus()
{
  ds402::Statusword word;
  word.all =  getTPDOValue<uint16_t>("status_word");
  return word;
}



}}//end of NS
