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
#include "CanOpenReceiveThread.h"


#include "Logging.h"

namespace icl_hardware {
namespace canopen_schunk {

CanOpenReceiveThread::CanOpenReceiveThread( const icl_core::TimeSpan& period,
                                const boost::shared_ptr<icl_hardware::can::tCanDevice>& can_device,
                                ReceivedPacketCallback const& received_callback)
  : m_period_time_ms(period.toMSec()),
    m_can_device (can_device),
    m_received_callback(received_callback)
{
  m_thread = boost::thread(&CanOpenReceiveThread::workerFunction, this);
}

CanOpenReceiveThread::~CanOpenReceiveThread()
{
  stop();
  m_thread.join();
}


void CanOpenReceiveThread::workerFunction()
{
  int32_t receive_result;
  while (true)
  {
    if (m_can_device)// != NULL)
    {
      if (m_can_device->IsInitialized())
      {
        receive_result = receiveData();
        if (receive_result == -ENODATA)
        {
          // currently do nothing
        }
        else if (receive_result != 0)
        {
          LOGGING_ERROR_C (CanOpen, CanOpenReceiveThread, "Reading CAN message failed, received error code " << receive_result << ". Doing nothing." << endl);
        }
      }
      else
      {
        LOGGING_WARNING_C(CanOpen, CanOpenReceiveThread, "Cannot read data from can device. It is not initialized!" << endl);
      }
    }

    // Wait for the thread period so that the timing is in sync.
    try
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(m_period_time_ms));
    }
    catch(boost::thread_interrupted&)
    {
        return;
    }
  }
}

void CanOpenReceiveThread::stop()
{
  m_thread.interrupt();
}

int32_t CanOpenReceiveThread::receiveData()
{
  int32_t receive_result = m_can_device->Receive(m_can_msg);
  if ( receive_result > 0)
  {
    m_received_callback(m_can_msg);
  }
  else
  {
    return receive_result;
  }
  return 0;
}



}} // end of NS
