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

#ifndef EMCY_H
#define EMCY_H

#include <boost/shared_ptr.hpp>
#include "helper.h"
#include "SDO.h"

namespace icl_hardware {
namespace canopen_schunk {

/*!
 * \brief The EMCY class handles the spontaneously occurring Emergency (EMCY) messages, keeps track of a nodes EMCY state and offers translation of the error codes to human readable form
 *
 * The EMCY class is used by the CanOpenNodes to store and retrieve the Emergency messages associated with a specific node. It handles incoming emcy messages and provides
 * the required calls to reset them. It also stores extensive lookup tables for the error messages. This EMCY class is meant as a base class for generic DS301 errors
 * and can be derived to produce more specific error messages regarding the used profile or specific device.
 */
class EMCY
{
public:
  typedef boost::shared_ptr<EMCY> Ptr;
  typedef boost::shared_ptr<const EMCY> ConstPtr;

  /*!
   * \brief DS301 requests state error_free and error_occurred. This could be mapped into a bool, but maybe
   * some other implementation needs more states, so we use an enum.
   */
  enum eEMCY_STATUS
  {
    EMCY_STATE_ERROR_FREE,
    EMCY_STATE_ERROR_OCCURED
  };

  static const uint16_t EMCY_ERROR_RESET_NO_ERROR = 0x0000;

  /*!
   * \brief EMCY Constructs a new EMCY object. The error map will be filled separately
   */
  EMCY(const uint8_t node_id);

  /*!
   * \brief update Updates the EMCY object with a received EMCY Message
   * \param msg Received Can Message that was already determined to be an EMCY message
   */
  virtual void update(const CanMsg& msg);

  /*!
   * \brief Returns the state of the EMCY state machine
   */
  eEMCY_STATUS getEmcyStatus() const;

  /*!
   * \brief Returns the full error information
   *
   * \param[out] eec Emergency error code
   * \param[out] error_register Error register
   * \param[out] msef Manifacturer specific error code
   * \return True if this device is in an erroneous state, false otherwise
   */
  bool getErrorInformation(uint16_t& eec, uint8_t& error_register, std::vector<uint8_t>& msef);

  /*!
   * \brief Adds new information from an ini file to the emergency error map
   *
   * \param filename Filename of the ini file
   * \param block_identifier Block identifier (the one in [] brackets)
   */
  static void addEmergencyErrorMap(const std::string& filename, const std::string& block_identifier);

  /*!
   * \brief Adds new information from an ini file to the error_register map
   *
   * \param filename Filename of the ini file
   * \param block_identifier Block identifier (the one in [] brackets)
   */
  static void addErrorRegisterMap(const std::string& filename, const std::string& block_identifier);

  /*!
   * \brief Print all errors present in error register 0x1003
   *
   * \param sdo Handle to the node's sdo object for communication
   * \throws Every exception that can occur in an SDO communication
   */
  void printLastErrors( SDO& sdo );

  /*!
   * \brief Prints a specific error from the error register 0x1003
   *
   * \note If the error under the given error_nr does not exist this will throw a
   * TransferAbortException.
   *
   * \param sdo Handle to the node's sdo object for communication
   * \param error_nr Subindex that should be printed
   *
   * \throws TransferAbortException if error_nr does not exist
   * \throws Every Exception that can occur during SDO downloads
   */
  void printError (SDO& sdo, const uint8_t error_nr = 1);

  /*!
   * \brief Clear the error register 0x1003
   *
   * \note This will set the EMCY state machine to \a error_free
   *
   * \param sdo Handle to the node's sdo object for communication
   * \throws Every exception that can occur in an SDO download
   */
  void clearErrorHistory(SDO& sdo);


protected:
  virtual std::string lookupMSEFString () const;

private:
  static std::string lookupEECString(const uint16_t error_code);
  static std::string lookupErrorRegisterString(const uint8_t error_code);

  uint8_t m_node_id;

  //! manufacturer-specific error code
  std::vector<uint8_t> m_msef;

  //! register in which the error occured
  uint8_t m_error_register;

  //! emergency_error_code;
  uint16_t m_eec;

  eEMCY_STATUS m_error_state;

  boost::mutex m_data_buffer_mutex;

  static std::map<uint16_t, std::string> m_eec_map;
  static std::map<uint8_t, std::string> m_error_register_map;
};


}} // end of NS

#endif // EMCY_H
