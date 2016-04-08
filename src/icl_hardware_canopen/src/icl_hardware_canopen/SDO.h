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

#ifndef SDO_H
#define SDO_H
#include <stdint.h>
#include "helper.h"
#include "exceptions.h"

namespace icl_hardware {
namespace canopen_schunk {

/*!
 * \brief The SDO class represents Service Data Objects (SDO) that are used for slow access of the canOpen object dictionary.
 * It uses the COB-ID which identifies entry with a 16bit index and 8 bit sub index and allows read and write operations.
 * The SDO is mainly used to set up the system before the actual cyclic operation starts.
 * This class holds the generic data, provides the up- and download functionality and parses incoming SDOs to ensure that they are correct
 */
class SDO
{
public:
  typedef boost::shared_ptr<SDO> Ptr;
  typedef boost::shared_ptr<const SDO> ConstPtr;


  /*!
   *  Standard base CAN IDs of SDO for transmit and receive:
   *
   *  Transmit/receive is as seen by the SCHUNK module (as usual for CANopen).
   *  => Transmit is Module to PC
   *  => Receive is PC to Module
   */
  static uint16_t const SDOTX_ID = 0x580; //             # 1408 -> 1409 ... 1535
  static uint16_t const SDORX_ID = 0x600; //             # 1536 -> 1537 ... 1663

  // download
  static unsigned char const SDO_SEG_REQ_INIT_DOWNLOAD_xBYTE = 0x22;
  static unsigned char const SDO_SEG_REQ_INIT_DOWNLOAD_1BYTE = 0x2F;
  static unsigned char const SDO_SEG_REQ_INIT_DOWNLOAD_2BYTE = 0x2B;
  static unsigned char const SDO_SEG_REQ_INIT_DOWNLOAD_3BYTE = 0x27;
  static unsigned char const SDO_SEG_REQ_INIT_DOWNLOAD_4BYTE = 0x23;
  static unsigned char const SDO_SEG_RES_INIT_DOWNLOAD = 0x60;
  // upload
  static unsigned char const SDO_SEG_REQ_INIT_UPLOAD = 0x40;
  static unsigned char const SDO_SEG_RES_INIT_UPLOAD_nBYTE =  0x41;
  static unsigned char const SDO_SEG_RES_INIT_UPLOAD_xBYTE =  0x42;
  static unsigned char const SDO_SEG_RES_INIT_UPLOAD_1BYTE = 0x4F;
  static unsigned char const SDO_SEG_RES_INIT_UPLOAD_2BYTE = 0x4B;
  static unsigned char const SDO_SEG_RES_INIT_UPLOAD_3BYTE = 0x47;
  static unsigned char const SDO_SEG_RES_INIT_UPLOAD_4BYTE = 0x43;
  static unsigned char const SDO_SEG_REQ_UPLOAD0 = 0x60; // used for segmented upload only
  static unsigned char const SDO_SEG_REQ_UPLOAD1 = 0x70; // used for segmented upload only

  static unsigned char const SDO_SEG_ABORT_TRANSFER =0x80;


  SDO(const uint8_t& node_id, const CanDevPtr& can_device);

  /*!
   * \brief update updates the SDO data with newly received messages
   * \param msg can message that was preciously identified as SDO message
   * \throws ProtocolException on errors
   */
  void update(const CanMsg& msg);

  /*!
   * \brief Downloads SDO data from the master to the slave (From PC to node).
   *
   * There exist expedited and normal (segmented) downloads in the DS301 standard.
   * Currently, only expedited downloads are supported, which send data of up to
   * 4 bytes directly with the download request.
   *
   * \param normal_transfer expedited (false) or segmented (true) downloads
   * \param index Index to where the data should be written (in the node)
   * \param subindex Subindex to where the data should be written (in the node)
   * \param usrdata Data that would be downloaded to the node.
   * \note: There also exists
   * an interface that accepts arbitrary datatypes that have a maximum size of 4 bytes.
   * \throws ProtocolException or any exception inherited from that on errors
   * \return True if download succeeds, false otherwise
   */
  bool download(const bool normal_transfer,
                const uint16_t index,
                const uint8_t subindex,
                const std::vector<uint8_t>& usrdata);

  /**
   * \brief Downloads SDO data from the master to the slave (From PC to node)
   *
   * This is just an overloaded interface to the vector interface. (see function definition
   * without template). It accepts all kind of userdata types that can fit into 4 bytes of
   * data. They will be automatically converted to a vector of bytes.
   *
   * * \param normal_transfer expedited (false) or segmented (true, not yet supported) downloads
   * \param index Index to where the data should be written (in the node)
   * \param subindex Subindex to where the data should be written (in the node)
   * \param usrdata Data that should be downloaded to the node. This can be any basic datatype
   * that fits into 4 bytes.
   * \throws ProtocolException or any exception inherited from that on errors
   * \return True if download succeeds, false otherwise
   */
  template <typename T>
  bool download(const bool normal_transfer,
                const uint16_t index,
                const uint8_t subindex,
                const T& usrdata)
  {
    std::vector<uint8_t> data_vector = convertToCharVector(usrdata);
    return download(normal_transfer, index, subindex, data_vector);
  }


  /*!
   * \brief Uploads data from a slave (node) to a master (PC).
   *
   * There exist expedited and normal (segmented) uploads in the DS301 standard.
   * Currently, only expedited uploads are supported, which send data of up to
   * 4 bytes directly with the first upload response.
   *
   * \param[in] normal_transfer expedited (false) or segmented (true, not yet supported) uploads
   * \param[in] index Index to where the data should be read (in the node)
   * \param[in] subindex Subindex to where the data should be read (in the node)
   * \param[out] uploaded_data uploaded data will be in this vector, if upload succeeds.
   * \throws ProtocolException or any exception inherited from that on errors
   * \return True if upload succeeds, false otherwise.
   */
  bool upload(const bool normal_transfer,
              const uint16_t index,
              const uint8_t subindex,
              std::vector<uint8_t>& uploaded_data);

  template <typename T>
  /*!
   * \brief Uploads data from a slave (node) to a master (PC).
   *
   * There exist expedited and normal (segmented) uploads in the DS301 standard.
   * Currently, only expedited uploads are supported, which send data of up to
   * 4 bytes directly with the first upload response.
   *
   * \param[in] normal_transfer expedited (false) or segmented (true, not yet supported) uploads
   * \param[in] index Index to where the data should be read (in the node)
   * \param[in] subindex Subindex to where the data should be read (in the node)
   * \param[out] uploaded_data uploaded data will be in here, if upload succeeds.
   * \throws ProtocolException or any exception inherited from that on errors
   * \return True if upload succeeds, false otherwise.
   */
  bool upload(const bool normal_transfer,
              const uint16_t index,
              const uint8_t subindex,
              T& uploaded_data)
  {
    std::vector<uint8_t> buffer;
    bool ret = upload (false, index, subindex, buffer);

    if (!ret || buffer.size() == 0)
    {
      throw ProtocolException (index, subindex, "Uploaded data was empty");
    }

    uploaded_data = convertFromCharVector<T>(buffer);

    // To be honest, this only can be true right now, otherwise we throw
    return ret;
  }

  /*!
   * \brief Adds an error map from an INI file to all SDOs. This should be called once to get
   * human readable errors.
   *
   * \param filename filename of the ini file
   */
  static void addErrorMapFromFile(const std::string& filename);

  /*!
   * \brief Set the time in milliseconds that should be waited for an SDO response
   * when performing a transfer.
   */
  void setResponseWaitTime(const uint32_t wait_time_ms) {m_response_wait_time_ms = wait_time_ms;}

  /*!
   * \brief Get the current SDO transfer response wait time
   */
  uint32_t getResponseWaitTime () const {return m_response_wait_time_ms;}

private:
  static std::string lookupErrorString(const uint32_t error_code);


  uint8_t m_node_id;
  CanDevPtr m_can_device;
  uint32_t m_response_wait_time_ms;

  bool m_data_update_received;
  boost::mutex m_data_buffer_mutex;
  boost::condition_variable m_data_buffer_updated_cond;
  std::vector<uint8_t> m_data_buffer;

  static std::map<uint32_t, std::string> m_error_map;
};

}}//end of NS

#endif // SDO_H
