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
 * \date    2015-10-1
 *
 */
//----------------------------------------------------------------------

#ifndef DS301NODE_H
#define DS301NODE_H

// CANOPEN inlcudes
#include "NMT.h"
#include "RPDO.h"
#include "TPDO.h"
#include "SDO.h"
#include "EMCY.h"
#include "HeartBeatMonitor.h"

#include "helper.h"
#include "Logging.h"
#include "exceptions.h"

#include <boost/unordered_map.hpp>

#ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
// Schunk Diagnostics addon
#include <icl_comm_websocket/WsBroadcaster.h>
#else
// Forward Deklaration of the WsBroadcaster
// This is not needed for normal driver operation
// but might be added later. To keep the interface the same
// a forward declaration becomes necessary
namespace icl_comm{
namespace websocket{
  class WsBroadcaster;
}}// NS end
#endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_


namespace icl_hardware {
namespace canopen_schunk {

/*!
 * \brief The DS301Node class Is the base class representation of canOpen devices. It is the access point to the most relevant functionalities of the DS301 protocol (canOpen)
 *
 * Each Node is a representation of a physical (or in the terms of canOpen also a logical) device. It holds information about the actual devices such
 * as its NMT state, the last EMCY message received from it or management information such as the ID of the device.
 * The DS301 Node implements the basic structure of canOpen nodes but is meant as a base class for derived classes implementing a specific protocol such as the DS402 motor protocol.
 */
class DS301Node
{
public:
  /*!
   * \brief Unique index to find a mapped Object dictionary item in a PDO.
   *
   * Mappings for RPDOs and TPDOs are separated
   *
   * The \a pdo_nr member describes the PDO's index in the node's pdo vector.
   * The \a pdo_mapping_index member stands for the internal vector index inside the PDO.
   */
  struct PDOMapEntry
  {
    uint16_t pdo_nr;
    uint8_t pdo_mapping_index;
  };

  /*!
   * \brief Type of a PDO. RECEIVE_PDOs carry data from the host to the device, TRANSMIT_PDOs
   * from the device to the host.
   */
  enum ePDO_TYPE
  {
    RECEIVE_PDO,
    TRANSMIT_PDO
  };

  //! Shared pointer to a DS301Node
  typedef boost::shared_ptr<DS301Node> Ptr;
  //! Shared pointer to a const DS301Node
  typedef boost::shared_ptr<const DS301Node> ConstPtr;

  DS301Node(const uint8_t node_id, const CanDevPtr& can_device, HeartBeatMonitor::Ptr heartbeat_monitor);

  uint8_t getNodeId() const {return m_node_id;}


  /*!
   * \brief Initializes the node
   */
  virtual void initNode();

  /*!
   * \brief registerWSBroadcaster Adds a debug interface
   */
  virtual void registerWSBroadcaster(boost::shared_ptr<icl_comm::websocket::WsBroadcaster> broadcaster);

  /*!
   * \brief Initializes the heartbeat for the node. Throws ProtocolException it heartbeat
   * cannot be initialized.
   */
  virtual void startHeartbeat();

  /*!
   * \brief Init PDO mapping with a given mapping configuration for a given pdo nr.
   *
   * \param config List of MappingConfigurations that should be mapped
   * \param pdo_nr Index of the PDO this mapping should be done.
   * \param transmission_type Transmission type of this PDO
   * \param pdo_type Is it a receive PDO or a transmit PDO?
   * \param dummy_mapping if set to True, no download to the device will be performed, but
   * the mapping will be done on host side. This is especially useful if a device has preconfigured
   * PDOs which you like to use.
   * \param cyclic_timeout_cycles If the transmission type SYNCHRONOUS_CYCLIC is used, this
   * parameter defines the PDO's frequency by defining the number of cycles between two send
   * attempts. For example, a parameter value of 4 means, that the PDO is sent every 5th cycle.
   * \throws PDOException when something goes wrong.
   */
  virtual void initPDOMappingSingle (const PDO::MappingConfigurationList& config,
                                     const uint16_t pdo_nr,
                                     const PDO::eTransmissionType& transmission_type,
                                     const ePDO_TYPE& pdo_type,
                                     const bool dummy_mapping = false,
                                     const uint8_t cyclic_timeout_cycles = 0);

  /*!
   * \brief Appends one or more mapping parameters to the existing mapping. Note that the PDO will
   * be disabled while appending another mapping.
   *
   * \param config List of MappingConfigurations that should be appended.
   * \param pdo_nr Index of the PDO this mapping should be done.
   * \param transmission_type Transmission type of this PDO
   * \param pdo_type Is it a receive PDO or a transmit PDO?
   * \param dummy_mapping if set to True, no download to the device will be performed, but
   * the mapping will be done on host side. This is especially useful if a device has preconfigured
   * PDOs which you like to use.
   * \param cyclic_timeout_cycles In case of transmission type SYNCHRONOUS_CYCLIC this
   * parameter defines the PDO's frequency by defining the number of cycles between two send
   * attempts. For example, a parameter value of 4 means, that the PDO is sent every 5th cycle.
   */
  virtual void appendPDOMappingSingle (const PDO::MappingConfigurationList& config,
                                      const uint16_t pdo_nr,
                                      const PDO::eTransmissionType& transmission_type,
                                      const ePDO_TYPE& pdo_type,
                                      const bool dummy_mapping = false,
                                      const uint8_t cyclic_timeout_cycles = 0);

  template <typename T>
  T getRPDOValue (const std::string& identifier)
  {
    // find pdo with this identifier
    if (m_rpdo_mapping.find(identifier) == m_rpdo_mapping.end())
    {
      std::stringstream ss;
      ss << "Could not find RPDO entry identifier string " << identifier << ". Aborting action now. ";
      throw PDOException(ss.str());
    }
    const PDOMapEntry& entry = m_rpdo_mapping[identifier];

    const PDO::Mapping& mapping = m_rpdos[entry.pdo_nr]->m_mapping_list[entry.pdo_mapping_index];
    // check if it is castable to the requested value

    return convertFromCharVector<T>(mapping.data);
  }

  /*!
   * \brief Get the value of a PDO which is mapped to a given identifier.
   *
   * \param identifier String identifier for the PDO
   * \throws PDOException when no PDO is found under the given identifier.
   * \return value of the PDO entry
   */
  template <typename T>
  T getTPDOValue (const std::string& identifier)
  {
    // find pdo with this identifier
    if (m_tpdo_mapping.find(identifier) == m_tpdo_mapping.end())
    {
      std::stringstream ss;
      ss << "Could not find TPDO entry identifier string " << identifier << ". Aborting action now. ";
      throw PDOException(ss.str());
    }
    const PDOMapEntry& entry = m_tpdo_mapping[identifier];

    const PDO::Mapping& mapping = m_tpdos[entry.pdo_nr]->m_mapping_list[entry.pdo_mapping_index];
    // check if it is castable to the requested value

    return convertFromCharVector<T>(mapping.data);
  }

  /*!
   * \brief Set the value of a PDO which is mapped to a given identifier.
   *
   * \param identifier String identifier for the PDO
   * \throws PDOException when no PDO is found under the given identifier.
   * \return True on success.
   */
  template <typename T>
  bool setRPDOValue (const std::string& identifier, const T value)
  {
    // find pdo with this identifier
    if (m_rpdo_mapping.find(identifier) == m_rpdo_mapping.end())
    {
      std::stringstream ss;
      ss << "Could not find RPDO entry identifier string " << identifier << ". Aborting action now. ";
      throw PDOException(ss.str());
    }
    const PDOMapEntry& entry = m_rpdo_mapping[identifier];
    PDO::Mapping& mapping = m_rpdos[entry.pdo_nr]->m_mapping_list[entry.pdo_mapping_index];

    if (sizeof(T) == mapping.data.size())
    {
      std::memcpy(&(mapping.data[0]), &value, sizeof(T));
    }

    LOGGING_TRACE (CanOpen, "Setting " << identifier << " for node " << m_node_id << " to " << value << endl);

    return true;
  }

  /*!
   * \brief Set the value of a PDO which is mapped to a given identifier.
   * \note The value set here will be overwritten by the next PDO
   *
   * \param identifier String identifier for the PDO
   * \throws PDOException when no PDO is found under the given identifier.
   * \return True on success.
   */
  template <typename T>
  bool setTPDOValue (const std::string& identifier, const T value)
  {
    // find pdo with this identifier
    if (m_tpdo_mapping.find(identifier) == m_tpdo_mapping.end())
    {
      std::stringstream ss;
      ss << "Could not find TPDO entry identifier string " << identifier << ". Aborting action now. ";
      throw PDOException(ss.str());
    }
    const PDOMapEntry& entry = m_tpdo_mapping[identifier];
    PDO::Mapping& mapping = m_tpdos[entry.pdo_nr]->m_mapping_list[entry.pdo_mapping_index];

    if (sizeof(T) == mapping.data.size())
    {
      std::memcpy(&(mapping.data[0]), &value, sizeof(T));
    }

    LOGGING_TRACE (CanOpen, "Setting " << identifier << " for node " << m_node_id << " to " << value << endl);

    return true;
  }

  /*!
   * \brief Uploads all Transmit-PDOs of this node from the device.
   */
  void uploadPDOs ();

  /*!
   * \brief Downloads all Receive-PDOs of this node to the device.
   */
  void downloadPDOs ();

  /*!
   * \brief Will query the PDO mapping from the device and print that to the output
   */
  void printPDOMapping();

  /*!
   * \brief This function maps a callback to a specific mapped PDO entry. This callback will
   * be called by the PDO upload function, whenever a PDO with the specified data will
   * arrive. Please note, that the callback function will only be notified of new data, but
   * the data must be queried from the PDO separately.
   * A PDO entry can call multiple callbacks, currently callbacks can't be unregistered.
   *
   * Throws a PDOException, when no PDO entry with the given identifier can be found.
   *
   * \param identifier string identifier of the mapped PDO entry
   * \param f notification callback function
   */
  void registerPDONotifyCallback (const std::string& identifier, const boost::function<void ()>& f );

  /*!
   * \brief Puts the node into nmt state preo_operational
   */
  virtual void stopNode();

  //! Object to handle NMT calls and status
  NMT m_nmt;
  //! SDO object for specific calls
  SDO m_sdo;
  //! RPDOS of this node (up to 4 in standard config)
  RPDO::PtrList m_rpdos;
  //! TPDOS of this node (up to 4 in standard config)
  TPDO::PtrList m_tpdos;
  //! EMCY object to handle spontaneous callbacks with emergency messages
  EMCY::Ptr m_emcy;

protected:
  //! CANOPEN ID of the node
  uint8_t m_node_id;

  //! Device handle for transmission of messages
  CanDevPtr m_can_dev;

  //! This map holds a mapping between an identifier string and a mapped position in a RPDO.
  boost::unordered_map<std::string, PDOMapEntry> m_rpdo_mapping;

  //! This map holds a mapping between an identifier string and a mapped position in a TPDO.
  boost::unordered_map<std::string, PDOMapEntry> m_tpdo_mapping;

  HeartBeatMonitor::Ptr m_heartbeat_monitor;

  uint16_t m_heartbeat_cycle_time_ms;

  //! Interface to send out diagnostics data. Only available if compiled with _IC_BUILDER_ICL_COMM_WEBSOCKET_
  boost::shared_ptr<icl_comm::websocket::WsBroadcaster> m_ws_broadcaster;

};

}}// end of NS

#endif // DS301NODE_H
