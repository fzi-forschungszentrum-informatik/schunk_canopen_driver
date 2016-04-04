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
* \date    2015-10-02
*
*/
//----------------------------------------------------------------------

#include "helper.h"
#include "Logging.h"

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

namespace icl_hardware{
namespace canopen_schunk{

std::string hexToString(const uint64_t num)
{
  std::stringstream ss;
  ss << "0x"
     << std::hex << std::setw(2) << std::setfill('0')
     << static_cast<int>( num );
  return ss.str();
}

std::string hexArrayToString( const unsigned char* msg, const uint8_t length)
{
  std::stringstream ss;
  for (size_t i=0; i < length; ++i)
  {
    ss << hexToString (msg[i]) << " ";
  }

  return ss.str();
}

std::string binaryString (const uint64_t num)
{
  std::bitset<64> bs(num);
  std::stringstream ss;
  ss << "0b" << bs;
     return ss.str();
}


std::string sanitizeString(const std::string& text)
{
  std::ostringstream ss;
  for (std::string::const_iterator it = text.begin(); it != text.end(); ++it)
  {
    if (std::isgraph(*it) && *it != '\r' && *it != '\n')
    {
      ss << *it;
    }
  }
  return ss.str();
}

std::map< uint32_t, std::string > getErrorMapFromConfigFile(const std::string& filename,
                                                            const std::string& category)
{
  std::map< uint32_t, std::string > ret_map;

  try
  {
    boost::property_tree::ptree tree;
    boost::property_tree::read_ini(filename, tree);

    BOOST_FOREACH(boost::property_tree::ptree::value_type &v,
                  tree.get_child(category))
    {

      // The data function is used to access the data stored in a node.
      uint32_t key = 0x0;
      std::stringstream ss;
      ss << std::hex << v.first.data();
      ss >> key;
      ret_map[key] = v.second.data();
    }
  }
  catch ( const boost::property_tree::ptree_error& e )
  {
    LOGGING_ERROR(CanOpen, "Error parsing error codes " << filename << ": "
                           << e.what()
                           << ". These error codes will not be available." << endl);
  }

  return ret_map;
}

}} // End of NS
