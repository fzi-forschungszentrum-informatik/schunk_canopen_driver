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

#ifndef PDO_H
#define PDO_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include "helper.h"
#include "SDO.h"

namespace icl_hardware {
namespace canopen_schunk {


/*!
 * \brief The PDO class provides access to one (of the possible multiple) Process Data Object of a canOpen node. The class provides structures for transmit and received data and the functions to trigger down- and uploads
 *
 * The class implements PDO access by providing data structures for the userdata and interfaces to trigger a down and upload. These functions will be called
 * periodically in order to transmit userdata to the actual devices and vice versa. The Class also offers the functionality to trigger a PDO remap which is done during
 * the setup phase of the devices. A node may hold 1-4 PDOs in the standard case but up to 512 in the extended case.
 */
class PDO
{
public:
  /*!
   * \brief Mapping of a PDO. This is basically a description that says where to look in the object
   * dictionary and how many bits to read
   * \note the \a length attribute describes the data length in Bits, not Bytes. An unsigned32
   * for example will be of length 0x20.
   */
  struct MappingConfiguration
  {
    uint16_t index;
    uint8_t subindex;
    uint8_t length;
    std::string name;
    /*!
     * \brief MappingConfiguration Create a new mapping configuration entry
     * \param index_ Object Dictionary Index
     * \param subindex_ Object Dictionary Sub-Index
     * \param length_ length in BITS not bytes. For exammple: An unsigned32 will have the length of 0x20
     * \param name_ Arbitrary name to identify and acces the pdo mapped value later on (for example: "speed")
     */
    MappingConfiguration (const uint16_t index_, const uint8_t subindex_, const uint8_t length_, const std::string& name_) :
    index(index_), subindex(subindex_), length(length_), name(name_) {}
  };
  //! The MappingConfigurationList holds multiple Mapping configurations. The Mapping of a single PDO is defined by one such configuration list
  typedef std::vector<MappingConfiguration> MappingConfigurationList;

  /*!
   * \brief Holds the mapping parameter plus the actual data.
   *
   * The Mapping of the PDO contains mapping information and data at the same time
   * Each PDO Stores a mapping configuration containing the individual entries of the PDO to transmit or Receive
   * These entries can be referenced by name so the user is able to get a specific value of the PDO by referring to it
   * by name. The Actuall data is stored in the data vector if the Mapping object.
   * This vector contains the data of all values mapped on this pdo in sequenc of their entries in the
   * Mapping Configurarion. Each PDO may transmit or receive up to 8 bytes of data.
   * The Mapping may be appended at a later time or completely destroyed with a new mapping.
   * \note Remapping of PDOs is only allowed if the NMT State is in pre operational.
   */
  class Mapping
  {
  public:
    /*!
     * \brief Mapping Creates a new mapping that stores the mapping information and the mapped data
     * \param mapping_configuration_ Description of the values to map
     */
    Mapping (const MappingConfiguration& mapping_configuration_) : mapping_configuration ( mapping_configuration_ )
    {
      data.resize( mapping_configuration.length / 8, 0);
    }

    //! Actual data of the PDO is stored in this data vector
    std::vector<uint8_t> data;
    /*!
     * \brief getConfiguration Returns the current mapping configuration of the PDO
     * \return Currently active mapping configuration
     */
    MappingConfiguration getConfiguration() const { return mapping_configuration;}
  private:
    MappingConfiguration mapping_configuration;
  };
  typedef std::vector<Mapping> MappingList;

  /*!
   * \brief Unique index to find a mapped Object dictionary item in a PDO.
   *
   * The \a name String identifier of the mapped register
   * The \a pdo_mapping_index Internal vector index inside the PDO.
   */
  struct PDOStringMatch
  {
    std::string name;
    uint8_t pdo_mapping_index;
  };
  typedef std::vector<PDOStringMatch> PDOStringMatchVec;

  //! Transmission types of a PDO, needed when mapping PDOs
  enum eTransmissionType {
    SYNCHRONOUS_ACYCLIC = 0,
    RTR_ONLY_SYNCHRONOUS = 252,
    RTR_ONLY_EVENT_DRIVEN = 253,
    EVENT_DRIVEN_MANUFACTURER_SPECIFIC = 254,
    EVENT_DRIVEN_PROFILE_SPECIFIC = 255,
    SYNCHRONOUS_CYCLIC = 1 // Note: There are many many more synchronous cyclic variants but as they are easily calculated we omit having enums for them
  };


  //! Convenience typedef to use PDOs with shared pointers
  typedef boost::shared_ptr<PDO> Ptr;
  //! Convenience typedef to use PDO lists with shared pointers
  typedef std::vector<boost::shared_ptr<PDO> > PtrList;

  /*!
   * \brief Construct a new PDO
   *
   * \param node_id ID of the node this PDO belongs to
   * \param pdo_nr numbering of this PDO inside the node
   * \param can_device handle to the CAN device
   */
  PDO(const uint8_t node_id, const uint8_t pdo_nr, const CanDevPtr& can_device);

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
   * \param pdo_cob_id CANOPEN-ID of this pdo
   * \param pdo_communication_parameter object dictionary entry of this pdo's communication parameter
   * \param pdo_mapping_parameter object dictionary entry of this pdo's mapping parameter
   * \param dummy_mapping if set to True, no download to the device will be performed, but
   * the mapping will be done on host side. This is especially useful if a device has preconfigured
   * PDOs which you like to use.
   * \param cyclic_timeout_cycles If the transmission type SYNCHRONOUS_CYCLIC is used, this
   * parameter defines the PDO's frequency by defining the number of cycles between two send
   * attempts. For example, a parameter value of 4 means, that the PDO is sent every 5th cycle.
   * \throws PDOException when an error occurs or an exception is thrown in an underlying
   * structure like SDO communication.
   * \return PDOStringMatchVec Vector of string to vec_index matchings. If an error occurs,
   * the returned vector will be empty.
   */
  PDOStringMatchVec remap (SDO& sdo,
                           const MappingConfigurationList& mappings,
                           const eTransmissionType& transmission_type,
                           const uint16_t pdo_cob_id,
                           const uint16_t pdo_communication_parameter,
                           const uint16_t pdo_mapping_parameter,
                           const bool dummy_mapping = false,
                           const uint8_t cyclic_timeout_cycles = 0);

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
   * \param pdo_cob_id CANOPEN-ID of this pdo
   * \param pdo_communication_parameter object dictionary entry of this pdo's communication parameter
   * \param pdo_mapping_parameter object dictionary entry of this pdo's mapping parameter
   * \param dummy_mapping if set to True, no download to the device will be performed, but
   * the mapping will be done on host side. This is especially useful if a device has preconfigured
   * PDOs which you like to use.
   * \param cyclic_timeout_cycles If the transmission type SYNCHRONOUS_CYCLIC is used, this
   * parameter defines the PDO's frequency by defining the number of cycles between two send
   * attempts. For example, a parameter value of 4 means, that the PDO is sent every 5th cycle.
   * \throws PDOException when an error occurs or an exception is thrown in an underlying
   * structure like SDO communication.
   * \return PDOStringMatchVec Vector of string to vec_index matchings. If an error occurs,
   * the returned vector will be empty.
   */
  PDOStringMatchVec appendMapping(SDO& sdo,
                                  const MappingConfigurationList& mappings,
                                  const eTransmissionType& transmission_type,
                                  const uint16_t pdo_cob_id,
                                  const uint16_t pdo_communication_parameter,
                                  const uint16_t pdo_mapping_parameter,
                                  const bool dummy_mapping = false,
                                  const uint8_t cyclic_timeout_cycles = 0);

  //! List of all mappings inside this PDO
  MappingList m_mapping_list;

protected:
  //! CANOPEN ID of the node this PDO belongs to
  uint8_t m_node_id;

  //! The PDO number inside the logical device. Theoretically this can be in 0 to 511
  uint8_t m_pdo_nr;

  //! Can Device handle
  CanDevPtr m_can_device;
};


}}// end of NS
#endif // PDO_H
