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

#ifndef SCHUNKPOWERBALLNODE_H
#define SCHUNKPOWERBALLNODE_H

#include "DS402Node.h"

namespace icl_hardware {
namespace canopen_schunk {

  /*!
   * \brief This class gives a device specific interface for Schunk Powerballs, as they need some
   * "special" treatment such as commutation search instead of homing.
   *
   * Also, the powerballs have predefined PDO mappings:
   * RPDO0 (Commands from PC to device):
   *  - "control_word" -> 0x6040
   *  - "torque_offset" -> 0x60B2
   *  - "interpolation_buffer" -> 0x60C1
   *
   * TPDO (Information from device to PC):
   *  - "status_word" -> 0x6041
   *  - "measured_torque" -> 0x6077 (which as far as we know are current values)
   *  - "measured_position" -> 0x6064
   *
   */
  class SchunkPowerBallNode : public DS402Node
{
public:
  //! Shared pointer to a SchunkPowerBallNode
  typedef boost::shared_ptr<SchunkPowerBallNode> Ptr;
  //! Shared pointer to a const SchunkPowerBallNode
  typedef boost::shared_ptr<const SchunkPowerBallNode> ConstPtr;

  /*!
   * \brief This factor will be used to convert RAD numbers into encoder ticks
   */
  static const double RAD_TO_STEPS_FACTOR = 57295.7795131;

  SchunkPowerBallNode(const uint8_t node_id, const icl_hardware::canopen_schunk::CanDevPtr& can_device, HeartBeatMonitor::Ptr heartbeat_monitor);

  /*!
   * \brief Performs a commutation search for a Schunk powerball module
   *
   * The commutation search gives information about where the module is when it is switched on.
   * Basically, this gets called automatically the first time a movement is requested from the device.
   *
   * Make sure, this is called during the initialization process (This should be taken care of by this
   * driver already)
   */
  void commutationSearch();

  /*!
   * \brief Set the device's homing method for a Schunk PowerBall
   *
   * Reimplementation for a Schunk powerball. It does not support to write the homing
   * method register 0x6098. Therefor this only sets the internal member variable to
   * the given value. Any value different from 0 enables the homing mode.
   * Apart from that, it's value will be ignored.
   *
   * \param homing_method Note that this parameter will be ignored.
   */
  virtual void configureHomingMethod (const uint8_t homing_method);

  /*!
   * \brief Set the speeds for homing for a Schunk PowerBall
   *
   * Reimplementation for a Schunk powerball. It does not support to write the homing
   * method register 0x6099.
   *
   * \param low_speed Note that this parameter will be ignored.
   * \param high_speed Note that this parameter will be ignored.
   */
  virtual void configureHomingSpeeds (const uint32_t low_speed, const uint32_t high_speed = 0);

  /*!
   * \brief Initializes the node and sets the Schunk Default PDO mapping.
   */
  virtual void initNode ();

  virtual void setDefaultPDOMapping (const DS402Node::eDefaultPDOMapping mapping);

protected:
  /*!
   * \brief Additionally to the basis implementation this sets a number of cycles that are allowed
   * to be missed by device before stopping execution
   *
   * See the basis implementation for member documentation.
   */
  virtual void configureInterpolationData (const uint8_t buffer_organization = 0,
                                           const int16_t interpolation_type = 0,
                                           const uint8_t size_of_data_record = 4);

private:
  /*!
   * \brief Checks if the device is already correctly commutated by checking the value of the
   * relevant register entry (via SDO communication).
   *
   * \return True, if commutation is already achieved, false otherwise.
   */
  bool CommutationCalibrated ();
};

}} // end of NS
#endif // SCHUNKPOWERBALLNODE_H
