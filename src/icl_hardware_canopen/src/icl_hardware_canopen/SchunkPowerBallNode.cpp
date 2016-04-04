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
* \date    2015-11-03
*
*/
//----------------------------------------------------------------------

#include "SchunkPowerBallNode.h"

#include "sync.h"

#include "Logging.h"
#include "exceptions.h"

namespace icl_hardware {
namespace canopen_schunk {

int boolify( int v )
{
    return (v!=0);
}

SchunkPowerBallNode::SchunkPowerBallNode (const uint8_t node_id, const CanDevPtr& can_device, HeartBeatMonitor::Ptr heartbeat_monitor)
  : DS402Node(node_id, can_device, heartbeat_monitor)
{
  m_homing_method = 33;
  m_transmission_factor = RAD_TO_STEPS_FACTOR;
}

void SchunkPowerBallNode::initNode()
{
  icl_hardware::canopen_schunk::DS402Node::initNode();
  m_nmt.start();
  commutationSearch();
  setModeOfOperation(ds402::MOO_PROFILE_POSITION_MODE);
}

void SchunkPowerBallNode::setDefaultPDOMapping (const DS402Node::eDefaultPDOMapping mapping)
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
      rpdo_mappings.push_back(PDO::MappingConfiguration(0x60B2, 0, 16, "torque_offset"));
      rpdo_mappings.push_back(PDO::MappingConfiguration(0x60C1, 1, 32, "interpolation_buffer"));

      tpdo_mappings.push_back(PDO::MappingConfiguration(0x6041, 0, 16, "status_word"));
      tpdo_mappings.push_back(PDO::MappingConfiguration(0x6077, 0, 16, "measured_torque"));
      tpdo_mappings.push_back(PDO::MappingConfiguration(0x6064, 0, 32, "measured_position"));

      initPDOMappingSingle (rpdo_mappings, 0, PDO::SYNCHRONOUS_CYCLIC, DS301Node::RECEIVE_PDO);
      initPDOMappingSingle (tpdo_mappings, 0, PDO::SYNCHRONOUS_CYCLIC, DS301Node::TRANSMIT_PDO);
      break;
    }
    case PDO_MAPPING_PROFILE_POSITION_MODE:
    {
      // We map the interpolated position buffer as well, as it is needed for commutation
      rpdo_mappings.push_back(PDO::MappingConfiguration(0x6040, 0, 16, "control_word"));
      rpdo_mappings.push_back(PDO::MappingConfiguration(0x60B2, 0, 16, "torque_offset"));
      rpdo_mappings.push_back(PDO::MappingConfiguration(0x60C1, 1, 32, "interpolation_buffer"));

      tpdo_mappings.push_back(PDO::MappingConfiguration(0x6041, 0, 16, "status_word"));
      tpdo_mappings.push_back(PDO::MappingConfiguration(0x6077, 0, 16, "measured_torque"));
      tpdo_mappings.push_back(PDO::MappingConfiguration(0x6064, 0, 32, "measured_position"));

      initPDOMappingSingle (rpdo_mappings, 0, PDO::SYNCHRONOUS_CYCLIC, DS301Node::RECEIVE_PDO);
      initPDOMappingSingle (tpdo_mappings, 0, PDO::SYNCHRONOUS_CYCLIC, DS301Node::TRANSMIT_PDO);

      rpdo_mappings.clear();
      rpdo_mappings.push_back(PDO::MappingConfiguration(0x607A, 0, 32, "target_position"));
      initPDOMappingSingle(rpdo_mappings, 1, PDO::SYNCHRONOUS_CYCLIC, DS301Node::RECEIVE_PDO);
      break;
    }
    default:
    {
      break;
    }
  }
}


void SchunkPowerBallNode::commutationSearch()
{
  LOGGING_INFO (CanOpen, "Commutation search for node  " << m_node_id << endl);

  bool calib_ok = CommutationCalibrated();

  // copy current position into the PDO buffer, as this will be read by the enable function
  int32_t current_position = 0;
  m_sdo.upload(false, 0x6064, 0x0, current_position);
  setTPDOValue("measured_position", current_position);
  LOGGING_INFO(CanOpen, "Initially, node is at position " << current_position << endl);

  // I'm not entirely sure why I need that, but I get an RPDO timeout error if I remove this

  sendSync(m_can_dev);
  bool disable_again = false;

  initDS402State(ds402::STATE_SWITCHED_ON);
  if (!calib_ok)
  {
    DS402Node::enableNode (ds402::MOO_INTERPOLATED_POSITION_MODE);
    disable_again = true;
    LOGGING_INFO (CanOpen, "Activated commutation search for node " << m_node_id << endl);
    // Make sure, the node is in interpolated position mode and enabled
  }

  size_t counter = 50;

  // Try commutation search for counter times
  while (!calib_ok && counter--)
  {
    usleep(100000); // sleep for 100 ms
    downloadPDOs();
    sendSync(m_can_dev);
    uploadPDOs();
    calib_ok = CommutationCalibrated();
  }
  if(!calib_ok)
  {
    LOGGING_ERROR (CanOpen, "Commutation of node "<< (int)(m_node_id) << " could not be ensured after 50 tries! Aborting... You probably should do a recalibration!" << endl);
  }
  else
  {
    if (disable_again)
    {
      disableNode();
    }
    LOGGING_INFO (CanOpen, "Commutation search for node " << m_node_id << " was successful!" << endl);
  }


  #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
  if (m_ws_broadcaster)
  {
    m_ws_broadcaster->robot->setJointHomed(calib_ok, m_node_id);
    m_ws_broadcaster->robot->setInputToRadFactor(1.0 / RAD_TO_STEPS_FACTOR);
  }
  #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_

}


bool SchunkPowerBallNode::CommutationCalibrated()
{
  uint8_t commutation_status;

  bool commutation_search_completed = true;

  //  - Bit 0 (Bit-Maske 0x01) mit der Bedeutung "commutation_search_completed"
  //  - Bit 1 (Bit-Maske 0x02) mit der Bedeutung "pseudo_absolute_position_verified"
  m_sdo.upload(false, 0x2050, 0, commutation_status);
//   LOGGING_INFO(CanOpen, "Commutation status: " << hexToString(commutation_status) << endl);
  commutation_search_completed = commutation_status & (1 << 0);

  return commutation_search_completed;
}



void SchunkPowerBallNode::configureHomingMethod (const uint8_t homing_method)
{
  LOGGING_ERROR (CanOpen, "configureHomingMethod called for a Schunk powerball node (id " <<
    m_node_id << "). " <<
    "However, the powerballs only support one homing mode so this request will be ignored." << endl
  );
}

void SchunkPowerBallNode::configureHomingSpeeds (const uint32_t low_speed, const uint32_t high_speed)
{
  LOGGING_ERROR (CanOpen, "configureHomingSpeeds called for a Schunk powerball node (id " <<
    m_node_id << "). " <<
    "However, the powerballs do not allow that, so this request will be ignored." << endl
  );
}

void SchunkPowerBallNode::configureInterpolationData (const uint8_t buffer_organization,
                                                      const int16_t interpolation_type,
                                                      const uint8_t size_of_data_record)
{
  DS402Node::configureInterpolationData (buffer_organization, interpolation_type, size_of_data_record);

  LOGGING_DEBUG (CanOpen, "Initializing interpolation data for Schunk power ball node with id " << m_node_id << endl);

  // Number of cycles that can be missed by the interpolator. If for more than this number of cycles
  // no new interpolation data is sent, the node will go into quick stop.
  // For hard realtime security use a value of 1 here.
  int8_t data8 = 20;
  m_sdo.download(false, 0x200e, 0, data8);
}


}} // end of NS
