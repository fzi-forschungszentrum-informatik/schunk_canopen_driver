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
* \date    2015-12-02
*
*/
//----------------------------------------------------------------------

#include "HeartBeatMonitor.h"
#include "Logging.h"

namespace icl_hardware {
namespace canopen_schunk {

HeartBeatMonitor::HeartBeatMonitor()
  : m_period_time_ms(100),
  m_running(false)
{
  start();
}

HeartBeatMonitor::~HeartBeatMonitor()
{
  stop();
  m_thread.join();
}

void HeartBeatMonitor::registerErrorCallback (const boost::function< void() >& f)
{
  m_error_function = f;
}


void HeartBeatMonitor::workerFunction()
{
  while (true)
  {
    icl_core::TimeStamp now_time = icl_core::TimeStamp::now();
    for (std::map<uint8_t, icl_core::TimeStamp>::iterator it = m_timestamp_record.begin();
         it != m_timestamp_record.end();
    ++it)
    {
      icl_core::TimeSpan time_passed = now_time - it->second;
      LOGGING_TRACE (CanOpen, "Time passed since last heartbeat from node " <<
                             it->first << ": " << time_passed.toMSec() << "ms" << endl);
      if (time_passed.toMSec() > m_period_time_ms)
      {
        LOGGING_ERROR_C (CanOpen, HeartBeatMonitor, "Missing heartbeat from node " <<
                         it->first << endl);
        m_error_function();
        stop();
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


void HeartBeatMonitor::addHeartbeat (const uint8_t node_id)
{
  if (m_running)
  {
    LOGGING_TRACE_C(CanOpen, HeartBeatMonitor, "New heartbeat received from node " << node_id << endl);

    m_timestamp_record[node_id] = icl_core::TimeStamp::now();
  }
}

void HeartBeatMonitor::stop()
{
  m_running = false;
  m_thread.interrupt();
}

void HeartBeatMonitor::start()
{
  if (!m_running)
  {
    m_thread = boost::thread(&HeartBeatMonitor::workerFunction, this);
    m_running = true;
  }
  else
  {
    LOGGING_WARNING(CanOpen, "start() called although HeartbeatMonitor thread is already running. Start request will be ignored." << endl);
  }
}

void HeartBeatMonitor::reset()
{
  // stop monitoring to prevent thread issues
  if (m_running)
  {
    stop();
  }

  // set all heartbeat times to zero
  for (std::map<uint8_t, icl_core::TimeStamp>::iterator it = m_timestamp_record.begin();
       it != m_timestamp_record.end();
      ++it)
  {
      it->second = icl_core::TimeStamp::now();
  }

  // restart monitoring
  start();
}





}} // End of NS
