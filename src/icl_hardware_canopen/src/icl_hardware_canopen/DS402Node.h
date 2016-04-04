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

#ifndef DS402NODE_H
#define DS402NODE_H

#include "DS301Node.h"

#include "ds402.h"


namespace icl_hardware {
namespace canopen_schunk {

  /*!
   * \brief Class that holds devices according to the DS402 (drives and motion control)
   * specification.
   *
   * It defines interfaces to different DS402 operation modes and offers many comfort
   * interface methods to acquire typical device operations such as  e.g. QuickStop triggering.
   */
  class DS402Node : public DS301Node
{
public:
  //! Shared pointer to a DS402Node
  typedef boost::shared_ptr<DS402Node> Ptr;
  //! Shared pointer to a const DS402Node
  typedef boost::shared_ptr<const DS402Node> ConstPtr;

  enum eDefaultPDOMapping
  {
    PDO_MAPPING_CONTROLWORD_STATUSWORD,
    PDO_MAPPING_INTERPOLATED_POSITION_MODE,
    PDO_MAPPING_PROFILE_POSITION_MODE
  };

  DS402Node(const uint8_t node_id, const icl_hardware::canopen_schunk::CanDevPtr& can_device, HeartBeatMonitor::Ptr heartbeat_monitor);

  virtual void setNMTState (const NMT::eNMT_State state, const NMT::eNMT_SubState sub_state);

  /*!
   * \brief Sets the target for the motor. What that target is, depends on the selected mode
   * of operation. The value will be clamped into the range value that is defined by the CanOpen
   * specification.
   *
   * It might be for example a position, a velocity or a torque.
   * For transmission to the device PDOs are used. Make sure that those PDOs are configured correctly
   *
   * \param target Target value for current mode of operation
   * \return True if target could be set, false otherwise
   */
  virtual bool setTarget ( const float target );

  /*!
   * \brief This sets the RPDO communication for enabling the movement after a target has been set.
   * Note, that sync and pdo-download has to be called in between setting the target and enabling movement.
   * Afterwards call acceptPPTarget() to enable setting following targets.
   */
  virtual void startPPMovement ();


  /*!
   * \brief After enabling PP movement, this enables accepting new targets again.
   * Remember to sync all nodes and download RPDOs before and after.
   */
  virtual void acceptPPTargets ();

  /*!
   * \brief Depending on the mode of operation, this will return the current position, velocity or
   * torque.
   *
   * \return double
   */
  virtual double getTargetFeedback();

  /*!
   * \brief Choose one of the predefined default mappings. Please see the enum \a eDefaultPDOMapping
   * for a list of available mappings. See the implementation for further details about the mappings.
   *
   * \param mapping Mapping that should be set.
   */
  virtual void setDefaultPDOMapping (const eDefaultPDOMapping mapping);

  /*!
   * \brief Perform homing for this node.
   */
  virtual void home();

  /*!
   * \brief Initializes a state of the DS402 state machine. Performs all necessary state transitions
   * until the target state is reached. However, it performs a maximum of \a m_max_number_of_state_transitions
   * transitions.
   *
   * \param requested_state Target state
   * \throws DeviceException if the device is in a fault state from which it can't recover
   */
  virtual void initDS402State( const icl_hardware::canopen_schunk::ds402::eState& requested_state );


  //! Get the current mode of operation
  ds402::eModeOfOperation getModeOfOperation () const { return m_op_mode; }


  /*!
   * \brief Sets and initializes a mode of operation. First it will check whether the
   * device supports the requested mode. So if you request a mode that the device does not
   * support, it just returns false and does nothing.
   *
   * \param op_mode Operation mode according to ds402::eModeOfOperation.
   * \return True if mode was successfully initialized, false otherwise.
   */
  virtual bool setModeOfOperation (const ds402::eModeOfOperation op_mode);

  /*!
   * \brief Tests whether a given mode of operation is supported by the device.
   * \note This function will only compare it to the cached information about supported modes.
   * The user has to call querySupportedDeviceModes() at some earlier point.
   *
   * \param op_mode Mode of operation that should be checked
   * \return True if mode is supported, false otherwise.
   */
  virtual bool isModeSupported (const ds402::eModeOfOperation op_mode);

  /*!
   * \brief Prints out all Modes on which the device claims to be able to operate on.
   * \note Make sure to call either initNode() or querySupportedDeviceModes() before calling
   * this function.
   */
  void printSupportedModesOfOperation ();

  /*!
   * \brief Prints a stringified version of the statusword to the logging system
   */
  virtual void printStatus();

  /*!
   * \brief This sends the controlword for performing a quick_stop
   */
  virtual void quickStop();

  /*!
   * \brief This redefines the basic stopNode function
   *
   * Currently this simply calls quickStop()
   */
  virtual void stopNode();

  /*!
   * \brief Initializes the node. Tries to query all required data from the device
   *
   */
  virtual void initNode();

  /*!
   * \brief Uploads all supported modes of operation from the device. and stores them in the
   * \a m_supported_modes member.
   */
  virtual void querySupportedDeviceModes();

  /*!
   * \brief Configure the mandatory parameters for a profile position mode
   *
   * \param config Struct of parameters used for profile position mode
   */
  virtual void setupProfilePositionMode (const ds402::ProfilePositionModeConfiguration& config);


  /*!
   * \brief The Transmission factor is used for converting radiant units into device ticks.
   * By default it's value is 1, so you can configure the conversion pipeline in the device.
   * However, if the device does not implement any conversion, this can be done on host side with
   * this factor.
   */
  double getTransmissionFactor () const { return m_transmission_factor; }

  /*!
   * \brief The Transmission factor is used for converting radiant units into device ticks.
   * By default it's value is 1, so you can configure the conversion pipeline in the device.
   * However, if the device does not implement any conversion, this can be done on host side with
   * this factor.
   */
  void setTransmissionFactor (const double transmission_factor) { m_transmission_factor = transmission_factor; }

  /*!
   * \brief Configure the mandatory parameters for a homing mode
   *
   * \param config Struct of mandatory parameters for a homing mode
   */
  virtual void setupHomingMode (const ds402::HomingModeConfiguration& config);

  /*!
   * \brief Configure the mandatory parameters for a profile velocity mode
   *
   * \param config Struct of mandatory parameters for a profile velocity mode
   */
  virtual void setupProfileVelocityMode (const ds402::ProfileVelocityModeConfiguration& config);

  /*!
   * \brief Configure the mandatory parameters for a profile torque mode
   *
   * \param config Struct of mandatory parameters for a profile torque mode
   */
  virtual void setupProfileTorqueMode (const ds402::ProfileTorqueModeConfiguration& config);


  /*!
   * \brief This function is used to recover from a fault state.
   *
   * It clears the error register and performs state transition 15.
   */
  virtual bool resetFault ();


  size_t getMaximumNumberOfStateTransitions () const { return m_max_number_of_state_transitions; }
  void setMaximumNumberOfStateTransitions (const size_t max_number_of_state_transitions) { m_max_number_of_state_transitions = max_number_of_state_transitions; }


  /*!
   * \brief Set the interpolation cycle time in milliseconds. If no time is given the default will be used
   *
   * \param interpolation_cycle_time_ms Interpolation cycle time in milliseconds
   */
  virtual void configureInterpolationCycleTime (const uint8_t interpolation_cycle_time_ms = 8);

  /*!
   * \brief Set the device's homing method through an SDO (OD 0x6098)
   */
  virtual void configureHomingMethod ( const int8_t homing_method );

  /*!
   * \brief Set the speeds for homing through SDO 6099.
   * Typically, a high speed is used when searching
   * for a home switch and the slow speed is used when searching for the index.
   */
  virtual void configureHomingSpeeds (const uint32_t low_speed, const uint32_t high_speed = 0);

  /*!
   * \brief Switches on the device and enters Operation Enabled mode with the given mode. If the
   * requested mode is InterpolatedPositionMode, interpolation will be started automatically.
   *
   * \param operation_mode DS402 Operation mode that should be used. If left blank, no
   * mode will be set. However, then you should make sure to set it in advance.
   */
  virtual void enableNode (const ds402::eModeOfOperation operation_mode = ds402::MOO_RESERVED_0);

  /*!
   * \brief Puts the device into STATE_SWITCHED_ON mode. If the device was in InterpolatedPositionMode,
   * interpolation will be stopped here, as well
   */
  virtual void disableNode ();

  /*!
   * \brief When the device is in OperationEnabled mode, this function enables the drive motion.
   *
   */
  virtual void openBrakes ();

  /*!
   * \brief When the device is in OperationEnabled mode, this function disable the drive motion.
   *
   */
  virtual void closeBrakes ();

  /*!
   * \brief Returns whether the device has reached it's recent target
   *
   * \return True, if current target is reached, false otherwise
   */
  virtual bool isTargetReached();

  virtual ds402::Statusword getStatus();


protected:
  /*!
   * \brief Set the device's maximum acceleration through an SDO (OD 0x60c5)
   */
  void configureMaxAcceleration (const uint32_t acceleration);
  /*!
   * \brief Set the device's maximum deceleration through an SDO (OD 0x60c6)
   */
  void configureMaxDeceleration (const uint32_t deceleration);

  /*!
   * \brief Set the device's profile velocity through an SDO (OD 0x6081)
   */
  void configureProfileVelocity (const uint32_t velocity);
  /*!
   * \brief Set the device's profile acceleration through an SDO (OD 0x6083)
   */
  void configureProfileAcceleration (const uint32_t acceleration);
  /*!
   * \brief Set the device's profile deceleration through an SDO (OD 0x6084)
   */
  void configureProfileDeceleration (const uint32_t deceleration);
  /*!
   * \brief Set the device's quick stop deceleration through an SDO (OD 0x6085)
   */
  void configureQuickStopDeceleration (const uint32_t deceleration);
  /*!
   * \brief Set the device's motion profile type through an SDO (OD 0x6086)
   */
  void configureMotionProfileType (const int16_t motion_type);

  /*!
   * \brief Set the device's homing acceleration through an SDO (OD 0x609A)
   */
  void configureHomingAcceleration (const uint32_t acceleration);

  /*!
   * \brief Set the device's sensor selection code through an SDO (OD 0x606a)
   */
  void configureSensorSelectionCode ( const int16_t sensor_selection_code );

  /*!
   * \brief Set the device's torque slope through an SDO (OD 0x6087)
   */
  void configureTorqueSlope ( const uint32_t torque_slope );
  /*!
   * \brief Set the device's torque profile type through an SDO (OD 0x6088)
   */
  void configureTorqueProfileType ( const int16_t torque_profile_type );

  /*!
   * \brief Configure the buffer for the interpolated position mode
   *
   * \param buffer_organization 0: FIFO buffer, 1: Ring buffer
   * \param interpolation_type Type of interpolation
   *          - -32768 ... -1 : manufacturer specific
   *          - 0: linear interpolation, 1: Ring buffer
   *          - +1 ... 32767: reserved
   * \param size_of_data_record Size of interpolation buffer. Should be 4 for linear interpolation
   */
  virtual void configureInterpolationData (const uint8_t buffer_organization = 0,
                                           const int16_t interpolation_type = 0,
                                           const uint8_t size_of_data_record = 4);

  /*!
   * \brief This function queries the two operation mode specific bits and turns them to a
   * human-readable string
   *
   * \param statusword the statusword
   * \return Stringified opMode specific status
   */
  std::string operationModeSpecificStatus ( const ds402::Statusword& statusword );

  /*!
   * \brief Performs a state transition in the DS402 state machine
   *
   * \param transition Transition that should be performed
   * \throws ProtocolException on error
   */
  void doDS402StateTransition (const ds402::eStateTransission transition);

  /*!
   * \brief This will be called when a new statusword PDO comes in. If the device's state
   * differs from the one expected, the local status will be changed accordingly.
   */
  virtual void onStatusWordUpdate ();

  //! supported modes of operation
  ds402::SupportedDriveModes m_supported_modes;

  //! The mode of operation of this device
  ds402::eModeOfOperation m_op_mode;

  ds402::eState m_current_ds402_state;
  ds402::eState m_expected_ds402_state;

  ds402::ProfilePositionModeConfiguration m_ppm_config;

  /*!
   * \brief Cycle time of interpolation mode
   */
  uint8_t m_interpolation_cycle_time_ms;
  size_t m_max_number_of_state_transitions;
  int8_t m_homing_method;
  double m_transmission_factor;
};


}}// end of NS
#endif // DS402NODE_H
