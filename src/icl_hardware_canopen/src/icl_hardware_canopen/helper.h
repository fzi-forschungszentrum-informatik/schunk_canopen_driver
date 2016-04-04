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
* \date    2015-10-01
*
*/
//----------------------------------------------------------------------

#ifndef HELPER_H
#define HELPER_H

#include <boost/shared_ptr.hpp>
#include "CanOpenReceiveThread.h"
#include "Logging.h"

namespace icl_hardware{
namespace canopen_schunk{

typedef boost::shared_ptr<CanOpenReceiveThread> CanOpenReceiveThreadPtr;
typedef boost::shared_ptr<icl_hardware::can::tCanDevice> CanDevPtr;
typedef icl_hardware::can::tCanMessage CanMsg;


/*!
 * \brief Converts a hexadecimal number into its string representation 0xXX
 *
 * \param num char with hexadecimal number
 * \return std::string string representation of num
 */
std::string hexToString (const uint64_t num);

/*!
 * \brief Transforms an array of unsigned chars into a string of Hex representations of those chars
 *
 * \param msg Array with bytes to print
 * \param length Number of entries in the array
 * \return std::string representation of the array
 */
std::string hexArrayToString( const unsigned char* msg, const uint8_t length);

template <typename T>
std::string binaryString (const T num)
{
  const size_t T_size = sizeof(T) * 8;
  std::bitset<T_size> bs(num);
  std::stringstream ss;
  ss << "0b" << bs;
  return ss.str();
}

/*!
 * \brief This function removes all non-graphical characters from the given string.
 *
 * All spaces, tabs and all special characters not in !"#$%&'()*+,-./:;<=>?@[\]^_`{|}~
 * will be removed.
 *
 * \param[in] text String that will be parsed
 * \return std::string sanitized string
 */
std::string sanitizeString(const std::string& text);


/*!
 * \brief Creates an error map from a given INI file.
 *
 * The outcoming std::map will have uint32_t as identifiers and strings as messages.
 * The file has to be structured as follows:
 *
 *   [error_codes]
 *   0xXXXXXXXX=Message string of error error_code
 *   ...
 *
 * the category "error_codes" can be any arbitrary identifier
 *
 * \param filename Filename of INI file
 * \param category Category string
 */
std::map<uint32_t, std::string> getErrorMapFromConfigFile(const std::string& filename,
                                                          const std::string& category = "error_codes");

/*!
 * \brief This little helper transforms any datatype that has a size of at most 4 Bytes
 * into a vector of uint8_t. If the data can't be converted, because its size is more than
 * 4 bytes, the vector will be empty
 *
 * \param value arbitrary datatype value
 * \return std::vector< uint8_t > Byte representation of value
 */
template <typename T>
inline std::vector<uint8_t> convertToCharVector(const T value)
{
  std::vector<uint8_t> out_vec;
  if (boost::is_fundamental<T>::value)
  {
    int32_t full_byte = 0xff;
    size_t num_bytes = sizeof(T);
    int32_t cast_int;

    if ( num_bytes <= 4)
    {
      std::memcpy(&cast_int, &value, num_bytes);
      for (size_t i = 0; i < num_bytes; ++i)
      {
        int32_t unary = cast_int & full_byte;
        out_vec.push_back(static_cast<uint8_t>( unary >> i*8));
        full_byte = full_byte << 8;
      }
    }
    else
    {
      LOGGING_ERROR (CanOpen, "The given data is too large. The maximum datatype size is 4 bytes." << endl);
      throw std::bad_cast();
    }
  }
  else
  {
    LOGGING_ERROR (CanOpen, "Only fundamental datatypes can be casted with the help of " << "this function. Fundamental types include integral, floating point and void types." << endl);
    throw std::bad_cast();
  }

  return out_vec;
}


/*!
 * \brief This converts a vector of uint8_t entries into another datatype with a matching byte length.
 * The target datatypes must be of the same size as the vector has entries. Otherwise a std::bad_cast
 * exception is thrown. So make sure to use a try/catch mechanism when using this function.
 *
 * \param vec Vector of uint8_t entries
 */
template <typename T>
inline T convertFromCharVector(const std::vector<uint8_t>& vec)
{
  T ret_val = 0;
 size_t num_bytes = sizeof(T);
  if (boost::is_fundamental<T>::value && vec.size() == num_bytes)
  {
    std::memcpy(&ret_val, &vec[0], num_bytes);
  }
  else
  {
    LOGGING_ERROR (CanOpen, "Only fundamental datatypes can be casted with the help of " << "this function. Fundamental types include integral, floating point and void types." << endl);
    throw std::bad_cast();
  }
  return ret_val;
}

}} // NS

#endif // HELPER_H
