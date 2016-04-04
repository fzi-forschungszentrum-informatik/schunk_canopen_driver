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

#ifndef DS402GROUP_H
#define DS402GROUP_H

#include "DS301Group.h"
#include "DS402Node.h"


namespace icl_hardware {
namespace canopen_schunk {

/*!
 * \brief The DS402Group class is the base class for canOpen devices implementing the DS402 device protocol (motors)
 *
 * The DS402Group provides interfaces to a group of canOpen defices implementing the DS402 device profile. This includes
 * mainly the management of the internal statemachine by NMT protocoll but also interfaces for inclusion into custom software
 * such as setTarget or getStatus which can be uses for cyclic commandation of target positions (or other values if so desired)
 * The DS402Group is intentionally kept generic in its interface as it is supposed to control various DS402 devices. If a more specific
 * interface is required for the devices it is intended that the developer inherits from this class and implements further interfaces.
 * By building on the DS301Group the most common functionalities of the canOpen protocol should be inherited
 */
class DS402Group : public DS301Group
{
public:
  //! Shared pointer to a DS402Group
  typedef boost::shared_ptr<DS402Group> Ptr;
  //! Shared pointer to a const DS402Group
  typedef boost::shared_ptr<const DS402Group> ConstPtr;

  DS402Group(const std::string& name = "");

  /*!
   * \brief Initializes the node. Tries to query all required data from the device
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void initNodes(const int16_t node_id = -1);

  /*!
   * \brief Prints a stringified version of the statusword to the logging system
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void printStatus(const int16_t node_id = -1);

  /*!
   * \brief Sets the target for the motor. What that target is, depends on the selected mode
   * of operation. The value will be clamped into the range value that is defined by the CanOpen
   * specification.
   *
   * It might be for example a position, a velocity or a torque.
   * For transmission to the device PDOs are used. Make sure that those PDOs are configured correctly
   *
   * \param targets Vector of targets for all nodes value for current mode of operation
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   * \return True if target could be set for all nodes, false otherwise
   */
  virtual bool setTarget ( const std::vector<float>& targets );

  /*!
   * \brief This sets the RPDO communication for enabling the movement after a target has been set.
   * Note, that sync and pdo-download has to be called in between setting the target and enabling movement.
   * Afterwards call acceptPPTarget() to enable setting following targets.
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void startPPMovement (const int16_t node_id = -1);

  /*!
   * \brief After enabling PP movement, this enables accepting new targets again.
   * Remember to sync all nodes and download RPDOs before and after.
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void acceptPPTargets (const int16_t node_id = -1);

  /*!
   * \brief Sets the target for the motor. What that target is, depends on the selected mode
   * of operation. The value will be clamped into the range value that is defined by
   * the CanOpen specification.
   *
   * It might be for example a position, a velocity or a torque.
   * For transmission to the device PDOs are used. Make sure that those PDOs are configured correctly
   *
   * \param target Target value for current mode of operation
   * \param node_id Node-ID of the node this target should be applied to
   * \return True if target could be set, false otherwise
   */
  virtual bool setTarget ( const float target, const uint8_t node_id );

  /*!
   * \brief Depending on the mode of operation, this will return the current position,
   * velocity or torque.
   *
   * \param[out] feedback Vector of target_feedback for all nodes in this group.
   * \note The entries in this vector don't necessarily represent the same physical
   * property, as nodes could be in different modes of operation.
   * \note: The order in this vector is the one, the nodes have been added to the group!
   */
  virtual void getTargetFeedback(std::vector<double>& feedback);

  /*!
   * \brief Choose one of the predefined default mappings. Please see the enum \a eDefaultPDOMapping
   * for a list of available mappings. See the implementation for further details about the mappings.
   *
   * \param mapping Mapping that should be set.
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void setDefaultPDOMapping (const DS402Node::eDefaultPDOMapping mapping, const int16_t node_id = -1);

  /*!
   * \brief Perform homing for nodes.
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void home(const int16_t node_id = -1);

  /*!
   * \brief Gets a vector of the Mode of operation of all nodes within this group
   *
   * \param[out] modes Vector of all modes of operation in this group.
   * \note: The order in this vector is the one, the nodes have been added to the group!
   */
  virtual void getModeOfOperation (std::vector<ds402::eModeOfOperation>& modes);

  /*!
   * \brief Sets and initializes a mode of operation. First it will check whether the
   * device supports the requested mode. So if you request a mode that the device does not
   * support, it just returns false and does nothing.
   *
   * \param op_mode Operation mode according to ds402::eModeOfOperation.
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   *
   * \return True if mode was successfully initialized for all nodes, false otherwise.
   */
  virtual bool setModeOfOperation (const ds402::eModeOfOperation op_mode, const int16_t node_id = -1);

  /*!
   * \brief Tests whether a given mode of operation is supported by the device.
   * \note This function will only compare it to the cached information about supported modes.
   * The user has to call querySupportedDeviceModes() at some earlier point.
   *
   * \param op_mode Mode of operation that should be checked
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   * \return True if mode is supported by all nodes, false otherwise.
   */
  virtual bool isModeSupported (const ds402::eModeOfOperation op_mode, const int16_t node_id = -1);

  /*!
   * \brief This sends the controlword for performing a quick_stop
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void quickStop (const int16_t node_id = -1);


  /*!
   * \brief Configure the mandatory parameters for a profile position mode
   *
   * \param config Struct of parameters used for profile position mode
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void setupProfilePositionMode (const ds402::ProfilePositionModeConfiguration& config, const int16_t node_id = -1);

  /*!
   * \brief Configure the mandatory parameters for a homing mode
   *
   * \param config Struct of mandatory parameters for a homing mode
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void setupHomingMode (const ds402::HomingModeConfiguration& config, const int16_t node_id = -1);

  /*!
   * \brief Configure the mandatory parameters for a profile velocity mode
   *
   * \param config Struct of mandatory parameters for a profile velocity mode
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void setupProfileVelocityMode (const ds402::ProfileVelocityModeConfiguration& config, const int16_t node_id = -1);

  /*!
   * \brief Configure the mandatory parameters for a profile torque mode
   *
   * \param config Struct of mandatory parameters for a profile torque mode
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void setupProfileTorqueMode (const ds402::ProfileTorqueModeConfiguration& config, const int16_t node_id = -1);

  /*!
   * \brief This function is used to recover from a fault state.
   *
   * It clears the error register and performs state transition 15.
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual bool resetFault (const int16_t node_id = -1);

  /*!
   * \brief Set the interpolation cycle time in milliseconds. If no time is given the default will be used
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   * \param interpolation_cycle_time_ms Interpolation cycle time in milliseconds
   */
  virtual void configureInterpolationCycleTime (const int16_t node_id = -1, const uint8_t interpolation_cycle_time_ms = 20);

  /*!
   * \brief Set the device's homing method through an SDO (OD 0x6098)
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   * \param homing_method Homing method index that should be used
   */
  virtual void configureHomingMethod ( const int8_t homing_method, const int16_t node_id = -1);

  /*!
   * \brief Set the speeds for homing through SDO 6099.
   * Typically, a high speed is used when searching
   * for a home switch and the slow speed is used when searching for the index.
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void configureHomingSpeeds (const uint32_t low_speed, const uint32_t high_speed = 0, const int16_t node_id = -1);

  /*!
   * \brief Switches on the device and enters Operation Enabled mode with the given mode. If the
   * requested mode is InterpolatedPositionMode, interpolation will be started automatically.
   *
   * \param operation_mode DS402 Operation mode that should be used. If left blank, no
   * mode will be set. However, then you should make sure to set it in advance (Which
   * might be done in initialization already).
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void enableNodes (const ds402::eModeOfOperation operation_mode = ds402::MOO_RESERVED_0,
                            const int16_t node_id = -1);

  /*!
   * \brief Puts the device into STATE_SWITCHED_ON mode. If the device was in InterpolatedPositionMode,
   * interpolation will be stopped here, as well
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void disableNodes (const int16_t node_id = -1);

  /*!
   * \brief When the device is in OperationEnabled mode, this function enables the drive motion.
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void openBrakes (const int16_t node_id = -1);

  /*!
   * \brief When the device is in OperationEnabled mode, this function disables the drive motion.
   *
   * \param node_id If left out or given a negative value, the function will be called
   * for all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void closeBrakes (const int16_t node_id = -1);

  /*!
   * \brief Returns whether the device has reached it's recent target
   *
   * \param[out] reached_single Boolean vector that contains the taget-reached status
   * for each node individually.
   *
   * \return True, if current target is reached for all nodes, false otherwise
   */
  virtual bool isTargetReached (std::vector<bool>& reached_single);

protected:
  /*!
   * \brief Creates a new node and adds it to the group
   *
   * \note This method is protected, as new nodes should be added through the controller only
   * (Which is why the controller is a friend class)
   *
   * \param node_id ID of the new node
   * \param can_device Shared pointer to the can device
   * \return shared pointer to the new \a DS301Node
   */
  template <typename NodeT>
  DS301Node::Ptr addNode(const uint8_t node_id, const CanDevPtr can_device, HeartBeatMonitor::Ptr heartbeat_monitor);

  std::vector<DS402Node::Ptr> m_ds402_nodes;

  // the add-node method should be callable from the controller
  friend class CanOpenController;

};

}}// end of NS

// include template implementations
#include "DS402Group.hpp"

#endif // DS402GROUP_H
