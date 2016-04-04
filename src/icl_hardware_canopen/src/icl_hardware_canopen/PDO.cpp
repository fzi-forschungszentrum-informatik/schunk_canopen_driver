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
 */
//----------------------------------------------------------------------

#include "ds301.h"
#include "PDO.h"

#include "Logging.h"
#include "exceptions.h"

#include <string.h>

namespace icl_hardware {
namespace canopen_schunk {

PDO::PDO(const uint8_t node_id,
         const uint8_t pdo_nr,
         const icl_hardware::canopen_schunk::CanDevPtr& can_device)
  : m_node_id(node_id),
    m_pdo_nr(pdo_nr),
    m_can_device(can_device)
{
}

PDO::PDOStringMatchVec PDO::remap (SDO& sdo,
                                   const MappingConfigurationList& mappings,
                                   const eTransmissionType& transmission_type,
                                   const uint16_t pdo_cob_id,
                                   const uint16_t pdo_communication_parameter,
                                   const uint16_t pdo_mapping_parameter,
                                   const bool dummy_mapping,
                                   const uint8_t cyclic_timeout_cycles
                                  )
{
  m_mapping_list.clear();
  PDO::PDOStringMatchVec ret_vec;

  // if no mapping is given, we leave the mapping as is.
  if (mappings.size() == 0)
  {
    LOGGING_INFO_C (CanOpen, PDO, "Remapping called with empty mapping. Not doing anything." << endl);
    return PDO::PDOStringMatchVec();
  }

  // A maximum of 64 mappings are supported.
  // See DS301 Specification Section 7.5.2.36 (Object 0x1600 to 0x17FF). Sub Index ranges to 0x40 resulting in 64 maximum mappable values
  if (mappings.size() > 64)
  {
    std::stringstream ss;
    ss << "Illegal number of mappings given. A maximum of 64 mappings is allowed. " <<
          "However, " << mappings.size() << " mappings were given.";
    throw PDOException (ss.str());
  }

  // set the full transmission type
  uint8_t transmission_type_int = static_cast<uint8_t>(transmission_type);
  if (transmission_type == SYNCHRONOUS_CYCLIC)
  {
    transmission_type_int += cyclic_timeout_cycles;
  }

  // establish rpdo communication in index 0x1400 / 0x01
  uint16_t index = pdo_communication_parameter;
  uint8_t subindex = 0x01;
  std::vector<uint8_t> data(4, 0);

  // disable this PDO
  data[0] = pdo_cob_id & 0xff;
  data[1] = pdo_cob_id >> 8;
  data[2] = 0x00;
  data[3] = 0x80;

  if (!dummy_mapping)
  {
    try
    {
      sdo.download(false, index, subindex, data);
    }
    catch (const std::exception& e)
    {
      std::stringstream ss;
      ss << "Downloading PDO communication object failed! ";
      ss << e.what();
      throw PDOException (ss.str());
    }
  }

  // clear PDO
  index = pdo_mapping_parameter;
  subindex = 0x00;
  uint8_t clear_data = 0;

  if (!dummy_mapping)
  {
    try
    {
      sdo.download(false, index, subindex, clear_data);
    }
    catch (const std::exception& e)
    {
      std::stringstream ss;
      ss << "Clearing PDO mapping failed! ";
      ss << e.what();
      throw PDOException (ss.str());
    }
  }


  // perform mapping
  index = pdo_mapping_parameter;
  subindex = 0;
  uint8_t cumul_length = 0;
  for (size_t i = 0; i < mappings.size(); ++i)
  {
    ++subindex;
    data[0] = mappings[i].length;
    data[1] = mappings[i].subindex;
    data[2] = mappings[i].index & 0xff;
    data[3] = mappings[i].index >> 8;
    cumul_length += mappings[i].length;

    PDOStringMatch match;
    match.name = mappings[i].name;
    match.pdo_mapping_index = i;
    ret_vec.push_back(match);

    if (cumul_length > 64)
    {
      throw PDOException ("The configured length of the PDO mapping is too big. To send a PDO in one CAN frame its size cannot be larger than 64 bit");

      //TODO: Currently this will leave a disabled PDO. We could as well use the PDO with the mappings so far and just drop a warning...
    }

    LOGGING_DEBUG (CanOpen, "Mapping " << hexToString(mappings[i].index) << " / " << static_cast<int>(mappings[i].subindex) << endl );
    if (!dummy_mapping)
    {
      sdo.download(false, index, subindex, data);
    }

    m_mapping_list.push_back(Mapping(mappings[i]));
  }

  index = pdo_mapping_parameter;
  subindex = 0x00;
  uint8_t num_mappings = mappings.size();

  if (!dummy_mapping)
  {
    try
    {
      sdo.download(false, index, subindex, num_mappings);
    }
    catch (const std::exception& e)
    {
      std::stringstream ss;
      ss << "Setting number of mappings failed! ";
      ss << e.what();
      throw PDOException (ss.str());
    }
  }

  // set PDO's transmission type
  index = pdo_communication_parameter;
  subindex = 0x02;
  if (!dummy_mapping)
  {
    try
    {
      sdo.download(false, index, subindex, transmission_type_int);
    }
    catch (const std::exception& e)
    {
      std::stringstream ss;
      ss << "Downloading PDO communication object failed! ";
      ss << e.what();
      throw PDOException (ss.str());
    }
  }

  // enable this PDO
  index = pdo_communication_parameter;
  subindex = 0x01;
  data[0] = pdo_cob_id & 0xff;
  data[1] = pdo_cob_id >> 8;
  data[2] = 0x00;
  data[3] = 0x00;

  if (!dummy_mapping)
  {
    try
    {
      sdo.download(false, index, subindex, data);
    }
    catch (const std::exception& e)
    {
      std::stringstream ss;
      ss << "Enabling PDO object failed! ";
      ss << e.what();
      throw PDOException (ss.str());
    }
  }

  LOGGING_DEBUG (CanOpen, "Remapping for node " << static_cast<int>(m_node_id) << " finished!" << endl);

  return ret_vec;
}


PDO::PDOStringMatchVec PDO::appendMapping (SDO& sdo,
                                   const MappingConfigurationList& mappings,
                                   const eTransmissionType& transmission_type,
                                   const uint16_t pdo_cob_id,
                                   const uint16_t pdo_communication_parameter,
                                   const uint16_t pdo_mapping_parameter,
                                   const bool dummy_mapping,
                                   const uint8_t cyclic_timeout_cycles)
{
  // Create MappingConfiguration from m_mapping_list
  MappingConfigurationList new_configuration;
  uint8_t cum_length = 0;
  for (MappingList::iterator it = m_mapping_list.begin(); it != m_mapping_list.end(); ++it)
  {
    new_configuration.push_back(it->getConfiguration());
    cum_length += it->getConfiguration().length;
  }

  // append new mapping to old mappingConfiguration
  uint8_t new_cum_length = 0;
  for (MappingConfigurationList::const_iterator it = mappings.begin(); it != mappings.end(); ++it)
  {
    new_cum_length += it->length;
    new_configuration.push_back(*it);
  }

  // Check, if there's enough space left in the PDO
  if (cum_length + new_cum_length > 64)
  {
    std::stringstream ss;
    ss <<  "The requested length of the PDO mapping is too big. "<<
           "To send a PDO in one CAN frame its size cannot be larger than 64 bit." <<
           "Please append this configuration to another PDO.";
    throw PDOException (ss.str());
  }

  // call remap
  return remap(sdo,
               new_configuration,
               transmission_type,
               pdo_cob_id,
               pdo_communication_parameter,
               pdo_mapping_parameter,
               cyclic_timeout_cycles);
}



}}//end of NS
