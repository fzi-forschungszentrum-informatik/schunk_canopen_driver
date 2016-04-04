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
 * \date    2015-10-01
 *
 */
//----------------------------------------------------------------------

#ifndef CANOPENRECEIVETHREAD_H
#define CANOPENRECEIVETHREAD_H

#include <boost/thread.hpp>

#include <icl_core/TimeSpan.h>

#include <icl_hardware_can/tCanMessage.h>
#include <icl_hardware_can/tCanDevice.h>

namespace icl_hardware {
namespace canopen_schunk {

//! definition of boost function callback for received packages
typedef boost::function<void (const icl_hardware::can::tCanMessage& msg)> ReceivedPacketCallback;

/*!
 * \brief The CanOpenReceiveThread class handles incoming canOpen messages
 *
 * The CanOpenReceiveThread uses the can device to listen for new messages. Once received it
 * processes the general message to ensure that it is a valid can message without errors. If so
 * the actual parsing of the message is handled by the CanOpenController class which is called via
 * callback.
 */
class CanOpenReceiveThread
{
public:
  /*!
   * \brief CanOpenReceiveThread constructor
   *
   * \param period Time periods between two can readings
   * \param can_device shared pointer to a can device
   * \param received_callback Handle to a callback function that processes the can messages
   */
  CanOpenReceiveThread( const icl_core::TimeSpan& period,
                        const boost::shared_ptr< icl_hardware::can::tCanDevice >& can_device,
                        const ReceivedPacketCallback& received_callback );

  /*!
   * \brief ~CanOpenReceiveThread Destructor
   */
  virtual ~CanOpenReceiveThread();

  //! stops thread execution
  void stop();

private:
  //! state machine processing received data
  void workerFunction();
  /*!
   * \brief receiveData Reads Data from the can driver and calls the data receivec callback if it was valid
   * \return 0 on succes, a value <0 if an error eccoured
   */
  int32_t receiveData();

  //! Cycle period of the receive thread
  int32_t m_period_time_ms;

  //! Can handle for message receiving
  boost::shared_ptr<icl_hardware::can::tCanDevice> m_can_device;

  //! Callback for parsing of correctly received can messages (protocoll will be evaluated there)
  ReceivedPacketCallback m_received_callback;

  //! Feedback thread handle
  boost::thread m_thread;

  //! Can message for storing and transmission of received data
  icl_hardware::can::tCanMessage m_can_msg;
};



}} // end of NS

#endif // CANOPENRECEIVETHREAD_H
