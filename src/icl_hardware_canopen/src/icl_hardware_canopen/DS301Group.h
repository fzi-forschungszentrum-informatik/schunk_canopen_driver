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

#ifndef DS301GROUP_H
#define DS301GROUP_H

#include "DS301Node.h"

namespace icl_hardware {
namespace canopen_schunk {

/*!
 * \brief The DS301Group class is the base Class for all canOpen device groups, providing basic interfaces to the DS301 functionality
 *
 * The DS301 Group provides the interface to access devices using the DS301 protocol (canOpen). It implements the interfaces
 * needs to set up devices of a group and to access the relevant functions, such as NMT to bring the nodes into the correct state.
 * While the DS301Group implements the basic calls that can be used for canOpen device management, it is intended to be used as a
 * Base class that is derived from by the profile specific groups that offer a hardware related interface
 */
class DS301Group
{
public:
  //! Shared pointer to a DS301Group
  typedef boost::shared_ptr<DS301Group> Ptr;
  //! Shared pointer to a const DS301Group
  typedef boost::shared_ptr<const DS301Group> ConstPtr;

  DS301Group(const std::string& name = "");

  /*!
   * \brief Returns the group's node vector
   */
  virtual std::vector<DS301Node::Ptr> getNodes() const {return m_nodes;}

  /*!
   * \brief Deletes a node with a given node id from the node list, if it is present.
   * If the node can't be found in this group, this function will return false;
   *
   * \param node_id CANOPEN-Id of node that should be deleted.
   * \return bool True if node was found and deleted, false if node was not found.
   */
  virtual bool deleteNodeFromId (const uint8_t node_id);

  /*!
   * \brief Deletes all nodes assigned to the group.
   *
   * \param[out] deleted_ids Vector of CANOPEN IDs of deleted objects
   */
  virtual void deleteNodes(std::vector<uint8_t>& deleted_ids);

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
   * \param node_id If left out or given a negative value, the mapping will be applied
   * to all nodes in the group, otherwise it will only be used for the given node-id.
   * \throws PDOException when something goes wrong.
   */
  virtual void initPDOMappingSingle (const PDO::MappingConfigurationList& config,
                                     const uint16_t pdo_nr,
                                     const PDO::eTransmissionType& transmission_type,
                                     const DS301Node::ePDO_TYPE& pdo_type,
                                     const bool dummy_mapping = false,
                                     const uint8_t cyclic_timeout_cycles = 0,
                                     const int16_t node_id = -1);


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
   * \param node_id If left out or given a negative value, the mapping will be applied
   * to all nodes in the group, otherwise it will only be used for the given node-id.
   */
  virtual void appendPDOMappingSingle (const PDO::MappingConfigurationList& config,
                                       const uint16_t pdo_nr,
                                       const PDO::eTransmissionType& transmission_type,
                                       const DS301Node::ePDO_TYPE& pdo_type,
                                       const bool dummy_mapping = false,
                                       const uint8_t cyclic_timeout_cycles = 0,
                                       const int16_t node_id = -1);

  void uploadPDOs ();

  void downloadPDOs ();

  /*!
   * \brief Returns the group's identifier string
   */
  std::string getName() const {return m_name;}

  /*!
   * \brief Will query the PDO mapping from the device and print that to the output
   *
   * \param node_id If no or negative id given, the function will be called on every node
   * within this group. If a positive node_id is given, only the specified node will be
   * affected.
   */
  void printPDOMapping (const uint8_t node_id = -1);


  /*!
   * \brief registerWSBroadcaster Adds a debug interface
   */
  void registerWSBroadcaster(boost::shared_ptr<icl_comm::websocket::WsBroadcaster> broadcaster);

protected:
  /*!
   * \brief Creates a new \a DS301Node and adds it to the group-
   *
   * \note This method is protected, as new nodes should be added through the controller only
   * (Which is why the controller is a friend class)
   *
   * \param node_id ID of the new node
   * \param can_device Shared pointer to the can device
   * \return shared pointer to the new \a DS301Node
   */
  template <typename NodeT>
  DS301Node::Ptr addNode(const uint8_t node_id, const CanDevPtr can_device, HeartBeatMonitor::Ptr heartbeat_monitor);

  std::vector<DS301Node::Ptr> m_nodes;
  std::string m_name;

  // the add-node method should be callable from the controller
  friend class CanOpenController;

  //! Interface to send out diagnostics data. Only available if compiled with _IC_BUILDER_ICL_COMM_WEBSOCKET_
  boost::shared_ptr<icl_comm::websocket::WsBroadcaster> m_ws_broadcaster;
};


}} // end of NS

// include template implementations
#include "DS301Group.hpp"

#endif // DS301GROUP_H
