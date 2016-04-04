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
 * \date    2015-10-01
 *
 * The ds301.h file contains common protocol information such as constants, enums and common helper functions
 * for components working with the DS301 protocol (canOpen)
 *
 */
//----------------------------------------------------------------------
#ifndef DS301_H
#define DS301_H

#include <stdint.h> // Exact data types

namespace icl_hardware {
namespace canopen_schunk {
namespace ds301{

  ///---------------------------
  ///      DS301 IDs
  /// T = TRANSMIT = From Device to PC (Client to Master)
  /// R = RECEIVE = From PC to Device (from Master to Client)
  /// All codes are given in HEX. Min ... Max ranges are a result of the
  /// Node ID which is encoded within. An EMCY Message of a canOpen node with id 3
  /// would result in the id 0x80 + 0x03 = 0x83 and so forth.
  ///--------------------------

  //!
  static const uint16_t ID_NMT        = 0x00;
  static const uint16_t ID_SYNC       = 0x80; // # 128
  static const uint16_t ID_EMCY_MIN   = 0x81; // # 129 ... 255
  static const uint16_t ID_EMCY_MAX   = 0xFF;
  static const uint16_t ID_TIME       = 0x100; // # 256

  // PDO1
  static const uint16_t ID_TPDO1_MIN  = 0x181;  // #  385 ... 511
  static const uint16_t ID_TPDO1_MAX  = 0x1FF;
  static const uint16_t ID_RPDO1_MIN  = 0x201;  // #  513 ... 639
  static const uint16_t ID_RPDO1_MAX  = 0x27F;

  //PDO2
  static const uint16_t ID_TPDO2_MIN  = 0x281;  // #  641 ... 767
  static const uint16_t ID_TPDO2_MAX  = 0x2FF;
  static const uint16_t ID_RPDO2_MIN  = 0x301;  // #  769 ... 895
  static const uint16_t ID_RPDO2_MAX  = 0x37F;

  //PDO3
  static const uint16_t ID_TPDO3_MIN  = 0x381;  // # 897 ... 1023
  static const uint16_t ID_TPDO3_MAX  = 0x3FF;
  static const uint16_t ID_RPDO3_MIN  = 0x401;  // # 1025 ... 1151
  static const uint16_t ID_RPDO3_MAX  = 0x47F;

  //PDO4
  static const uint16_t ID_TPDO4_MIN  = 0x481;  // # 1153 ... 1279
  static const uint16_t ID_TPDO4_MAX  = 0x4FF;
  static const uint16_t ID_RPDO4_MIN  = 0x501;  // # 1281 ... 1407
  static const uint16_t ID_RPDO4_MAX  = 0x57F;

  //SDO
  static const uint16_t ID_TSDO_MIN  = 0x581; // # 1409 ... 1535
  static const uint16_t ID_TSDO_MAX  = 0x5FF;
  static const uint16_t ID_RSDO_MIN  = 0x601; // # 1537 ... 1663
  static const uint16_t ID_RSDO_MAX  = 0x67F;

  static const uint16_t ID_NMT_ERROR_MIN  = 0x701; // # 1793 ... 1919
  static const uint16_t ID_NMT_ERROR_MAX  = 0x77F;



}}} // end of NS

#endif // DS301_H
