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

#ifndef HEARTBEATMONITOR_H
#define HEARTBEATMONITOR_H

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <map>

#include <icl_core/TimeStamp.h>

namespace icl_hardware{
namespace canopen_schunk{

class HeartBeatMonitor
{
public:
  //! Shared pointer to a HeartBeatMonitor
  typedef boost::shared_ptr<HeartBeatMonitor> Ptr;
  //! Shared pointer to a const HeartBeatMonitor
  typedef boost::shared_ptr<const HeartBeatMonitor> ConstPtr;

  HeartBeatMonitor();

  ~HeartBeatMonitor();

  /*!
   * \brief Incoming heartbeats will trigger this function to update the most recent
   * heartbeat time.
   *
   * \param node_id Node-ID the heartbeat message belonged to.
   */
  void addHeartbeat(const uint8_t node_id);


  /*!
   * \brief This function will be called as soon as one node is missing a heartbeat
   * message. Probably this function should turn off / stop all devices.
   */
  void registerErrorCallback(const boost::function<void()>& f);

  /*!
   * \brief The HeartbeatMonitor checks for missing heartbeat messages with this cycle time.
   * If the last heartbeat message for one node was received more than one cycle ago, the
   * error function will be triggered. So basically, this cycle time should be greater than
   * the smallest heartbeat time.
   *
   * \return The cycle time in milliseconds
   */
  uint16_t getHeartBeatCycleTime () const { return m_period_time_ms; }

  /*!
   * \brief The HeartbeatMonitor checks for missing heartbeat messages with this cycle time.
   * If the last heartbeat message for one node was received more than one cycle ago, the
   * error function will be triggered. So basically, this cycle time should be greater than
   * the smallest heartbeat time.
   *
   * \param heartbeat_cycle_time_ms The cycle time in milliseconds
   */
   void setHeartBeatCycleTime (const uint16_t heartbeat_cycle_time_ms) { m_period_time_ms = heartbeat_cycle_time_ms; }

   /*!
    * \brief Resets the heartbeat monitor. This will make the monitor forget about all past
    * heartbeats and start from scratch.
    */
   void reset();

private:
  /*!
   * \brief Thread worker function for the Heartbeat monitor. This function will regularly
   * check for missing heartbeat messages. If a heartbeat message is missing,
   * \a m_error_function will be called.
   */
  void workerFunction();

  //! stops thread execution
  void stop();

  //! starts thread execution
  void start();


  std::map<uint8_t, icl_core::TimeStamp> m_timestamp_record;

  //! Cycle period of the monitor thread
  uint16_t m_period_time_ms;

  //! monitor thread handle
  boost::thread m_thread;

  boost::function<void()> m_error_function;

  bool m_running;
};

}} // End of NS

#endif // HEARTBEATMONITOR_H
