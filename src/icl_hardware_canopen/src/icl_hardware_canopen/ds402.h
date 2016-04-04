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
 *The ds402.h file contains common protocol information such as constants, enums and common helper functions
 * for components working with the DS402 device protocol (canOpen protocol for motor drives)
 * This includes data types for the status and control word that provide a central element for device configuration
 *
 */
//----------------------------------------------------------------------

#ifndef DS402_H
#define DS402_H

#include "Logging.h"

namespace icl_hardware {
namespace canopen_schunk {
namespace ds402{

static const uint16_t ID_CONTROL_WORD   = 0x6040;
static const uint16_t ID_STATUS_WORD    = 0x6041;

enum eModeOfOperation
{
  //-1 ... -128 Manufacturer specific modes of operation
  MOO_RESERVED_0                 = 0,
  MOO_PROFILE_POSITION_MODE      = 1,
  MOO_VELOCITY_MODE              = 2, // not supported right now
  MOO_PROFILE_VELOCITY_MODE      = 3,
  MOO_PROFILE_TORQUE_MODE        = 4,
  MOO_RESERVED_1                 = 5,
  MOO_HOMING_MODE                = 6,
  MOO_INTERPOLATED_POSITION_MODE = 7,
  MOO_CYCLIC_SYNC_POSITION_MODE  = 8,
  MOO_CYCLIC_SYNC_VELOCITY_MODE  = 9,
  MOO_CYCLIC_SYNC_TORQUE_MODE    = 10
  //11 ... 127 reserved
};

/*!
 * \brief DS402 states as described in Figure 6.3 in ELMO DS402 implementation guide V1.000
 */
enum eState
{
  // Do NOT change the order of these states!
  STATE_START,
  STATE_NOT_READY_TO_SWITCH_ON,
  STATE_SWITCH_ON_DISABLED,
  STATE_READY_TO_SWITCH_ON,
  STATE_SWITCHED_ON,
  STATE_OPERATION_ENABLE,
  STATE_QUICKSTOP_ACTIVE,
  STATE_FAULT_REACTION_ACTIVE,
  STATE_FAULT // This does not show up in the diagram, but it is acquired from the statusword
};

enum eStateTransission
{
  STATE_TRANS_INITIALIZE = 1,             // 1, 7, 9, 10, 12
  STATE_TRANS_SHUTDOWN = 2,               // 2, 6, 8
  STATE_TRANS_SWITCH_ON = 3,              // 3, 5
  STATE_TRANS_ENABLE_OPERATION = 4,       // 4, 16
  STATE_TRANS_QUICK_STOP = 11,            // 11
  STATE_TRANS_FAULT_RESET = 15            // 15
  // 0, 13 and 14 cannot be performed manually
};

//! bit field for DSP 402 6040 controlword,
struct Controlword_
{
  uint16_t switch_on                 : 1; // 0
  uint16_t enable_voltage            : 1; // 1
  uint16_t quick_stop                : 1; // 2
  uint16_t enable_operation          : 1; // 3
  uint16_t operation_mode_specific_0 : 1; // 4
  uint16_t operation_mode_specific_1 : 1; // 5
  uint16_t operation_mode_specific_2 : 1; // 6
  uint16_t reset_fault               : 1; // 7
  uint16_t halt                      : 1; // 8
  uint16_t reserved_0                : 1; // 9
  uint16_t reserved_1                : 1; // 10
  uint16_t manufacturer_specific_0   : 1; // 11
  uint16_t manufacturer_specific_1   : 1; // 12
  uint16_t manufacturer_specific_2   : 1; // 13
  uint16_t manufacturer_specific_3   : 1; // 14
  uint16_t manufacturer_specific_4   : 1; // 15
};

//! data union for access to DSP 402 6040 controlword,
union Controlword
{
  Controlword_ bit;
  uint16_t all;
};

//! bit field for DSP 402 6041 statusword,
struct Statusword_
{
  uint16_t ready_to_switch_on        : 1; // 0
  uint16_t switched_on               : 1; // 1
  uint16_t operation_enabled         : 1; // 2
  uint16_t fault                     : 1; // 3
  uint16_t voltage_enabled           : 1; // 4
  uint16_t quick_stop                : 1; // 5
  uint16_t switch_on_disabled        : 1; // 6
  uint16_t warning                   : 1; // 7
  uint16_t manufacturer_specific_0   : 1; // 8
  uint16_t remote                    : 1; // 9
  uint16_t target_reached            : 1; // 10
  uint16_t internal_limit_active     : 1; // 11
  uint16_t operation_mode_specific_0 : 1; // 12
  uint16_t operation_mode_specific_1 : 1; // 13
  uint16_t manufacturer_specific_1   : 1; // 14
  uint16_t manufacturer_specific_2   : 1; // 15
};

//! data union for access to DSP 402 6041 statusword,
union Statusword
{
  Statusword_ bit;
  uint16_t all;
};

struct SupportedDriveModes_
{
  uint32_t profile_position_mode      : 1; // 0
  uint32_t velocity_mode              : 1; // 1
  uint32_t profile_velocity_mode      : 1; // 2
  uint32_t profile_torque_mode        : 1; // 3
  uint32_t reserved_4                 : 1; // 4
  uint32_t homing_mode                : 1; // 5
  uint32_t interpolated_position_mode : 1; // 6
  uint32_t cyclic_sync_position_mode  : 1; // 7
  uint32_t cyclic_sync_velocity_mode  : 1; // 8
  uint32_t cyclic_sync_torque_mode    : 1; // 9
  // 10..15 reserved
  // 16..31 manufacturer specific
};

//! data union for access to DSP 402 6041 statusword,
union SupportedDriveModes
{
  SupportedDriveModes_    bit;
  uint32_t all;
};

/*!
 * \brief Configuration parameters for a Profile_Position_Mode according to CiA DSP-402 V1.1
 * section 12.2.1
 */
struct ProfilePositionModeConfiguration
{
  /*!
   * \brief Final velocity
   */
  float profile_velocity;

  /*!
   * \brief This will be used for both acceleration ramps
   */
  float profile_acceleration;

  /*!
   * \brief This will be used for both acceleration ramps
   * \note Some devices do not support setting the deceleration parameter. Acceleration parameter
   * will be used in this case. Also, this parameter defaults to 0, in which case also the acceleration
   * parameter will be used.
   *
   */
  float profile_deceleration;

  /*!
   * \brief Type of ramp used for accelerating and decelerating. Defaults to linear.
   */
  int16_t motion_profile_type;

  /*!
   * \brief If this is set to true the device will not ramp down at a setpoint if a following one
   * is given. Defaults to false.
   */
  bool change_set_immediately;

  /*!
   * \brief This parameter influences the interpretation of new set points. If set to true, new set point
   * positions will be interpreted as relative positions, if set to false absolute positions shall
   * be commanded. Defaults to false.
   *
   */
  bool use_relative_targets;

  /*!
   * \brief If set to true, the device will blend over to a new setpoint. Defaults to true.
   *
   */
  bool use_blending;


  ProfilePositionModeConfiguration()
    : profile_deceleration(0),
      motion_profile_type(0),
      change_set_immediately(false),
      use_relative_targets(false),
      use_blending(true)
    {}
};

/*!
 * \brief Configuration parameters for a Homing_Mode according to CiA DSP-402 V1.1
 * section 13.2.1
 */
struct HomingModeConfiguration
{
  int8_t homing_method;
  uint32_t homing_speed_low;
  uint32_t homing_speed_high;

  HomingModeConfiguration()
    : homing_method(0), homing_speed_low(0), homing_speed_high(0) {}

  HomingModeConfiguration(int8_t method_, uint32_t speed_low_, uint32_t speed_high)
    : homing_method( method_ ), homing_speed_low(speed_low_), homing_speed_high(speed_high) {}
};

/*!
 * \brief Configuration parameters for a Profile_Velocity_Mode according to CiA DSP-402 V1.1
 * section 16.2.1
 */
struct ProfileVelocityModeConfiguration
{
  int16_t sensor_selection_code;

  enum eSensorCode
  {
    POSITION_ENCODER = 0,
    VELOCITY_ENCODER = 1
  };
};

/*!
 * \brief Configuration parameters for a Profile_Torque_Mode according to CiA DSP-402 V1.1
 * section 17.2.1
 */
struct ProfileTorqueModeConfiguration
{
  uint32_t torque_slope;
  int16_t torque_profile_type;
  ProfileTorqueModeConfiguration()
    : torque_slope(0), torque_profile_type(0) {}
};

/*!
 * \brief Turns a status word into a status string according to the elmo DS-402 implementation guide
 * chapter 6.6.1
 *
 * \param statusword Statusword of device
 * \return stringified status
 */
inline std::string deviceStatusString (const eState state)
{
  switch (state)
  {
    case STATE_NOT_READY_TO_SWITCH_ON:
    {
      return "NOT READY TO SWITCH ON";
      break;
    }
    case STATE_SWITCH_ON_DISABLED:
    {
      return "SWITCH ON DISABLED";
      break;
    }
    case STATE_READY_TO_SWITCH_ON:
    {
      return "READY TO SWITCH ON";
      break;
    }
    case STATE_SWITCHED_ON:
    {
      return "SWITCHED ON";
      break;
    }
    case STATE_OPERATION_ENABLE:
    {
      return "OPERATION ENABLED";
      break;
    }
    case STATE_QUICKSTOP_ACTIVE:
    {
      return "QUICK STOP ACTIVE";
      break;
    }
    case STATE_FAULT_REACTION_ACTIVE:
    {
      return "FAULT REACTION ACTIVE";
      break;
    }
    case STATE_FAULT:
    {
      return "FAULT";
      break;
    }
    default:
    {
      // nothing. Will reach statement at end of function
    }
  }
  return "Unknown status code. This should not happen!";
}

inline std::string operationModeString (const eModeOfOperation mode)
{
  switch (mode)
  {
    case MOO_HOMING_MODE:
    {
      return "HOMING_MODE";
    }
    case MOO_INTERPOLATED_POSITION_MODE:
    {
      return "MOO_INTERPOLATED_POSITION_MODE";
    }
    case MOO_PROFILE_POSITION_MODE:
    {
      return "MOO_PROFILE_POSITION_MODE";
    }
    case MOO_PROFILE_TORQUE_MODE:
    {
      return "MOO_PROFILE_TORQUE_MODE";
    }
    case MOO_VELOCITY_MODE:
    {
      return "MOO_VELOCITY_MODE";
    }
    case MOO_PROFILE_VELOCITY_MODE:
    {
      return "MOO_PROFILE_VELOCITY_MODE";
    }
    case MOO_CYCLIC_SYNC_POSITION_MODE:
    {
      return "MOO_CYCLIC_SYNC_POSITION_MODE";
    }
    case MOO_CYCLIC_SYNC_VELOCITY_MODE:
    {
      return "MOO_CYCLIC_SYNC_VELOCITY_MODE";
    }
    case MOO_CYCLIC_SYNC_TORQUE_MODE:
    {
      return "MOO_CYCLIC_SYNC_TORQUE_MODE";
    }
    default:
    {
      return "UNKNOWN_MODE_OF_OPERATION";
    }
  }
}

inline eState stateFromStatusword (const ds402::Statusword& statusword)
{
  uint8_t lsb = statusword.all & 0xFF;
  if ( (lsb | 0b10110000) == 0b10110000)
  {
    return STATE_NOT_READY_TO_SWITCH_ON;
  }
  else if ( (lsb | 0b10110000) == 0b11110000)
  {
    return STATE_SWITCH_ON_DISABLED;
  }
  else if ( (lsb | 0b10010000) == 0b10110001)
  {
    return STATE_READY_TO_SWITCH_ON;
  }
  else if ( (lsb | 0b10010000) == 0b10110011)
  {
    return STATE_SWITCHED_ON;
  }
  else if ( (lsb | 0b10010000) == 0b10110111)
  {
    return STATE_OPERATION_ENABLE;
  }
  else if ( (lsb | 0b10010000) == 0b10010111)
  {
    return STATE_QUICKSTOP_ACTIVE;
  }
  else if ( (lsb | 0b10110000) == 0b10111111)
  {
    return STATE_FAULT_REACTION_ACTIVE;
  }
  else if ( (lsb | 0b10110000) == 0b10111000)
  {
    return STATE_FAULT;
  }

  // if we reach this point, something is wrong!
  LOGGING_ERROR (CanOpen, "Failed to get DS402 state from statusword! Something is probably wrong with the statusword. Returning STATE_FAULT" << endl);
  return STATE_FAULT;
}

}}} // end of NS
#endif // DS402_H
