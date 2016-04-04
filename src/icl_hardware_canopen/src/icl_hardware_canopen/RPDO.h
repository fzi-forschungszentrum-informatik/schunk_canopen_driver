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

#ifndef RPDO_H
#define RPDO_H

#include "PDO.h"

namespace icl_hardware {
namespace canopen_schunk {

/*!
 * \brief This class describes Receive PDOs, meaning PDOs that send data from the host to the device.
 */
class RPDO : public PDO
{
public:
  //! Convenience typedef to use PDOs with shared pointers
  typedef boost::shared_ptr<RPDO> Ptr;
  //! Convenience typedef to use PDO lists with shared pointers
  typedef std::vector<boost::shared_ptr<RPDO> > PtrList;

  static const uint16_t OD_RPDO_COMMUNICATION_MIN = 0x1400;
  static const uint16_t OD_RPDO_MAPPING_PARAMETER_MIN = 0x1600;

  /*!
   * \brief Construct a new RPDO
   *
   * \param node_id ID of the node this RPDO belongs to
   * \param pdo_nr numbering of this RPDO inside the node
   * \param can_device handle to the CAN device
   */
  RPDO(const uint8_t node_id, const uint8_t pdo_nr, const CanDevPtr& can_device);

  /*!
   * \brief Downloads RPDO data from the master to the slave (From PC to node)
   *
   * \param usrdata Data vector that should be transferred to the node.
   * \return bool True if download succeeds, false otherwise
   */
  bool download();


  /*!
   * \brief Configure a PDO by sending some SDO packages. This can be either done during
   * NMT state pre-operational or during the NMT state Operational.
   * If an empty mapping is given, we leave the mapping as is.
   *
   * \note Do not call this method by hand, but use the wrapper functions in the DS301Node class.
   * Otherwise the node will not know of the new mapping and won't be able to find a PDO by it's
   * identifier string.
   *
   * \param sdo Handle to the SDO object
   * \param mappings List of MappingConfigurations that should be mapped into this PDO
   * \param transmission_type Transmission type of this PDO
   * \param dummy_mapping if set to True, no download to the device will be performed, but
   * the mapping will be done on host side. This is especially useful if a device has preconfigured
   * PDOs which you like to use.
   * \param cyclic_timeout_cycles If the transmission type SYNCHRONOUS_CYCLIC is used, this
   * parameter defines the PDO's frequency by defining the number of cycles between two send
   * attempts. For example, a parameter value of 4 means, that the PDO is sent every 5th cycle.
   * \return PDOStringMatchVec Vector of string to vec_index matchings. If an error occurs,
   * the returned vector will be empty.
   */
  PDOStringMatchVec remap (SDO& sdo,
                           const MappingConfigurationList& mappings,
                           const eTransmissionType& transmission_type,
                           const bool dummy_mapping = false,
                           const uint8_t cyclic_timeout_cycles = 0
                          );

  /*!
   * \brief Appends one or more mapping parameters to the existing mapping. Note that the PDO will
   * be disabled while appending another mapping.
   *
   * \note Do not call this method by hand, but use the wrapper functions in the DS301Node class.
   * Otherwise the node will not know of the new mapping and won't be able to find a PDO by it's
   * identifier string.
   *
   * \param sdo Handle to the SDO object
   * \param mappings List of MappingConfigurations that should be mapped into this PDO
   * \param transmission_type Transmission type of this PDO
   * \param dummy_mapping if set to True, no download to the device will be performed, but
   * the mapping will be done on host side. This is especially useful if a device has preconfigured
   * PDOs which you like to use.
   * \param cyclic_timeout_cycles If the transmission type SYNCHRONOUS_CYCLIC is used, this
   * parameter defines the PDO's frequency by defining the number of cycles between two send
   * attempts. For example, a parameter value of 4 means, that the PDO is sent every 5th cycle.
   * \return PDOStringMatchVec Vector of string to vec_index matchings. If an error occurs,
   * the returned vector will be empty.
   */
  PDOStringMatchVec appendMapping(SDO& sdo,
                                  const MappingConfigurationList& mappings,
                                  const eTransmissionType& transmission_type,
                                  const bool dummy_mapping = false,
                                  const uint8_t cyclic_timeout_cycles = 0
                                 );
};

}} // end of NS

#endif // RPDO_H
