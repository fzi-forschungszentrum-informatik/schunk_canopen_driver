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
 * \date    2015-10-23
 *
 */
//----------------------------------------------------------------------

#ifndef ICL_HARDWARE_CANOPEN_EXCEPTIONS_H_INCLUDED
#define ICL_HARDWARE_CANOPEN_EXCEPTIONS_H_INCLUDED

#include <exception>
#include <iostream>
#include "helper.h"

namespace icl_hardware{
namespace canopen_schunk{

/*!
 * \brief Basic CanOpen exception that contains the Object dictionary index and subindex.
 */
class ProtocolException : public std::exception
{
public:
  ProtocolException(const uint16_t index,
                    const uint8_t subindex,
                    const std::string& error_msg = "none")
    : index_(index),
      subindex_(subindex),
      error_msg_(error_msg)
  {}

  virtual ~ProtocolException(void) _GLIBCXX_USE_NOEXCEPT {}

  virtual const char* what() const _GLIBCXX_USE_NOEXCEPT
  {
    std::stringstream ss;
    ss << "A protocol error occurred at index " << hexToString(index_) << ", subindex " <<
    hexToString(subindex_) << ". Additional information: " << error_msg_ << std::endl;
    return ss.str().c_str();
  }


protected:
  uint16_t index_;
  uint8_t subindex_;
  std::string error_msg_;
};

/*!
 * \brief Exceptions relating to device responses.
 */
class ResponseException : public ProtocolException
{
public:
  ResponseException(const uint16_t index,
                    const uint8_t subindex,
                    const std::string& error_msg = "none")
    : ProtocolException (index, subindex, error_msg)
  {}

  virtual ~ResponseException(void) _GLIBCXX_USE_NOEXCEPT {}

  virtual const char* what() const _GLIBCXX_USE_NOEXCEPT
  {
    std::stringstream ss;
    ss << "An invalid response was received for request at index " << hexToString(index_) <<
    ", subindex " << hexToString(subindex_) << ". Additional information: " << error_msg_;
    return ss.str().c_str();
  }
};

/*!
 * \brief If a device response times out, this exception will be thrown.
 */
class TimeoutException : public ProtocolException
{
public:
  TimeoutException(const uint16_t index,
                    const uint8_t subindex,
                    const std::string& error_msg = "none")
    : ProtocolException (index, subindex, error_msg)
  {}

  virtual ~TimeoutException(void) _GLIBCXX_USE_NOEXCEPT {}

  virtual const char* what() const _GLIBCXX_USE_NOEXCEPT
  {
    std::stringstream ss;
    ss << "Timeout while waiting for response at index  " << hexToString(index_) <<
    ", subindex " << hexToString(subindex_) << ". Additional information: " << error_msg_;
    return ss.str().c_str();
  }
};

/*!
 * \brief PDO related exceptions go here.
 */
class PDOException : public std::exception
{
public:
  PDOException(const std::string& error_msg = "none")
    : m_error_msg (error_msg)
  {}

  virtual ~PDOException(void) _GLIBCXX_USE_NOEXCEPT {}

  virtual const char* what() const _GLIBCXX_USE_NOEXCEPT
  {
    return m_error_msg.c_str();
  }

protected:
  std::string m_error_msg;
};

/*!
 * \brief If something goes wrong with the host's CAN controller, this exception will be used.
 */
class DeviceException : public std::exception
{
public:
  DeviceException(const std::string& error_string)
    : m_error_string(error_string)
  {}

  virtual ~DeviceException(void) _GLIBCXX_USE_NOEXCEPT {}

  virtual const char* what() const _GLIBCXX_USE_NOEXCEPT
  {
    std::stringstream ss;
    ss << m_error_string << " Check your configuration and make sure the device " <<
                    "is properly connected.";
    return ss.str().c_str();
  }

protected:
  std::string m_error_string;
};

/*!
 * \brief This exception is thrown if a requested node or node group does not exist.
 */
class NotFoundException : public std::exception
{
public:
  NotFoundException(const std::string& error_string)
    : m_error_string(error_string)
  {}

    virtual ~NotFoundException(void) _GLIBCXX_USE_NOEXCEPT {}

  virtual const char* what() const _GLIBCXX_USE_NOEXCEPT
  {
    return m_error_string.c_str();
  }

protected:
  std::string m_error_string;
};

}} // end of NS

#endif
