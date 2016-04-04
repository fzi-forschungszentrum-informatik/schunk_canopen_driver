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
 * \date    2015-10-7
 *
 */
//----------------------------------------------------------------------

#include <icl_hardware_can/tCanDeviceDummy.h>

#include <icl_hardware_canopen/CanOpenController.h>
#include <icl_hardware_canopen/DS301Group.h>
#include <icl_hardware_canopen/SDO.h>

#include <icl_hardware_canopen/Logging.h>
#include <icl_hardware_canopen/exceptions.h>

#include <boost/test/unit_test.hpp>
#include <boost/thread/thread.hpp>
using namespace icl_hardware::canopen_schunk;


BOOST_AUTO_TEST_SUITE(ts_sdo)

BOOST_AUTO_TEST_CASE (sdo_download)
{
  // Initializing
  icl_core::logging::initialize();
  icl_core::logging::setLogLevel(icl_core::logging::eLL_TRACE);
  LOGGING_INFO(CanOpen, "-----------------------------------" << endl <<
                        "-----Running SDO download test-----" << endl <<
                        "-----------------------------------" << endl);

  CanOpenController my_controller("Dummy");
  boost::shared_ptr<icl_hardware::can::tCanDeviceDummy> can_device;
  can_device = boost::dynamic_pointer_cast<icl_hardware::can::tCanDeviceDummy>(my_controller.getCanDevice());

  // Add one node
  my_controller.addGroup<DS301Group>("testgroup");
  my_controller.addNode<DS301Node>(1, "testgroup");
  DS301Group::Ptr my_group = my_controller.getGroup<DS301Group>("testgroup");
  DS301Node::Ptr node = my_group->getNodes().front();


  // Create some data that should be downloaded to the slave node
  std::vector<uint8_t> data(4);
  data[0] = 0x00;
  data[1] = 0x01;
  data[2] = 0x02;
  data[3] = 0x03;
  uint32_t index = 300;
  uint8_t subindex = 3;

  // create node response
  CanMsg response;
  response.id = ds301::ID_TSDO_MIN; // SDO for first node
  response.dlc = 8;
  response.rtr = 0;
  response.data[0] = SDO::SDO_SEG_RES_INIT_DOWNLOAD;
  response.data[1] = index & 0xff;
  response.data[2] = index >> 8;
  response.data[3] = subindex & 0xff;
  can_device->addResponse(response);

  // If download succeeds and the response is correct, this will pass
  BOOST_REQUIRE(node->m_sdo.download(false, index, subindex, data));

  // parse the sent can message that was created by the download request
  CanMsg msg = can_device->getLastMessage();
  LOGGING_INFO(CanOpen, hexArrayToString(msg.data, msg.dlc) << endl);
  BOOST_REQUIRE(msg.dlc == 8);
  BOOST_REQUIRE(msg.rtr == 0);
  BOOST_REQUIRE(msg.data[0] == SDO::SDO_SEG_REQ_INIT_DOWNLOAD_4BYTE);
  BOOST_REQUIRE(msg.data[1] == (index & 0xff));
  BOOST_REQUIRE(msg.data[2] == (index >> 8));
  BOOST_REQUIRE(msg.data[3] == subindex);
  BOOST_REQUIRE(msg.data[4] == data[0]);
  BOOST_REQUIRE(msg.data[5] == data[1]);
  BOOST_REQUIRE(msg.data[6] == data[2]);
  BOOST_REQUIRE(msg.data[7] == data[3]);
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // Now let's generate some faulty responses. They should all fail.
  response.dlc = 13; // some arbitrary illegal number
  can_device->addResponse(response);

  BOOST_REQUIRE_THROW(node->m_sdo.download(false, index, subindex, data), std::exception);
  response.dlc = 8;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";


  // This will produce a timeout as the message will never reach the SDO
  response.id = ds301::ID_TSDO_MIN-1;
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW(node->m_sdo.download(false, index, subindex, data), TimeoutException);
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // This will also produce a timout, as the node to this sdo does not exist
  response.id = ds301::ID_TSDO_MIN+1;
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW(node->m_sdo.download(false, index, subindex, data), TimeoutException);
  response.id = ds301::ID_TSDO_MIN;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // This will produce a wrong response error
  response.data[0] = SDO::SDO_SEG_RES_INIT_DOWNLOAD | 0x01;
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW(node->m_sdo.download(false, index, subindex, data), ResponseException);
  response.data[0] = SDO::SDO_SEG_RES_INIT_DOWNLOAD;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // LSB byte of index is wrong
  response.data[1] = (index+1) & 0xff;
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW(node->m_sdo.download(false, index, subindex, data), ResponseException);
  response.data[1] = index & 0xff;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // MSB byte of index is wrong
  response.data[2] = (index+256) >> 8;
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW(node->m_sdo.download(false, index, subindex, data), ResponseException);
  response.data[2] = index >> 8;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // Subindex is wrong
  response.data[3] = (subindex+1) & 0xff;
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW(node->m_sdo.download(false, index, subindex, data), ResponseException);
  response.data[3] = subindex & 0xff;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // blocked downloads are not yet supported
  can_device->addResponse(response);
  try {
    BOOST_REQUIRE(!(node->m_sdo.download(true, index, subindex, data)));
  }
  catch (const std::exception& e)
  {
    LOGGING_ERROR_C (CanOpen, SDO, e.what() << endl);
  }
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // illegal number of sent bytes
  can_device->addResponse(response);
  std::vector<uint8_t> empty_data;
  BOOST_REQUIRE_THROW (node->m_sdo.download(false, index, subindex, empty_data), ProtocolException);
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // illegal number of sent bytes
  can_device->addResponse(response);
  empty_data.resize(5);
  try {
    BOOST_REQUIRE(!(node->m_sdo.download(false, index, subindex, empty_data)));
  }
  catch (const std::exception& e)
  {
    LOGGING_ERROR_C (CanOpen, SDO, e.what() << endl);
  }
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";
}


BOOST_AUTO_TEST_CASE (sdo_download_test_interfaces)
{
  // Initializing
  icl_core::logging::initialize();
  icl_core::logging::setLogLevel(icl_core::logging::eLL_TRACE);
  LOGGING_INFO(CanOpen, "---------------------------------------------" << endl <<
                        "-----Running SDO download interface test-----" << endl <<
                        "---------------------------------------------" << endl);

  CanOpenController my_controller("Dummy");
  boost::shared_ptr<icl_hardware::can::tCanDeviceDummy> can_device;
  can_device = boost::dynamic_pointer_cast<icl_hardware::can::tCanDeviceDummy>(my_controller.getCanDevice());

  // Add one node
  my_controller.addGroup<DS301Group>("testgroup");
  my_controller.addNode<DS301Node>(1, "testgroup");
  DS301Group::Ptr my_group = my_controller.getGroup<DS301Group>("testgroup");
  DS301Node::Ptr node = my_group->getNodes().front();


  // Create some data that should be downloaded to the slave node
  std::vector<uint8_t> data(4);
  data[0] = 0x00;
  data[1] = 0x01;
  data[2] = 0x02;
  data[3] = 0x03;
  uint32_t index = 300;
  uint8_t subindex = 3;

  // create node response
  CanMsg response;
  response.id = ds301::ID_TSDO_MIN; // SDO for first node
  response.dlc = 8;
  response.rtr = 0;
  response.data[0] = SDO::SDO_SEG_RES_INIT_DOWNLOAD;
  response.data[1] = index & 0xff;
  response.data[2] = index >> 8;
  response.data[3] = subindex & 0xff;
  can_device->addResponse(response);

  BOOST_REQUIRE_NO_THROW(node->m_sdo.download(false, index, subindex, data));
  // parse the sent can message that was created by the download request
  CanMsg msg1 = can_device->getLastMessage();

  can_device->addResponse(response);
  uint32_t data_block = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + data[0];
  LOGGING_INFO (CanOpen, hexToString(data_block) << endl);
  BOOST_REQUIRE_NO_THROW(node->m_sdo.download(false, index, subindex, data_block));

  // parse the sent can message that was created by the download request
  CanMsg msg2 = can_device->getLastMessage();
  LOGGING_INFO(CanOpen, hexArrayToString(msg2.data, msg2.dlc) << endl);

  BOOST_REQUIRE(msg2.dlc == 8);
  BOOST_REQUIRE(msg2.rtr == 0);
  BOOST_REQUIRE(msg2.data[1] == (index & 0xff));
  BOOST_REQUIRE(msg2.data[0] == SDO::SDO_SEG_REQ_INIT_DOWNLOAD_4BYTE);
  BOOST_REQUIRE(msg2.data[2] == (index >> 8));
  BOOST_REQUIRE(msg2.data[3] == subindex);
  BOOST_REQUIRE(msg2.data[4] == data[0]);
  BOOST_REQUIRE(msg2.data[5] == data[1]);
  BOOST_REQUIRE(msg2.data[6] == data[2]);
  BOOST_REQUIRE(msg2.data[7] == data[3]);

  BOOST_REQUIRE(msg1.data[4] == msg2.data[4]);
  BOOST_REQUIRE(msg1.data[5] == msg2.data[5]);
  BOOST_REQUIRE(msg1.data[6] == msg2.data[6]);
  BOOST_REQUIRE(msg1.data[7] == msg2.data[7]);
}

BOOST_AUTO_TEST_CASE (sdo_upload)
{
  // Initializing
  icl_core::logging::initialize();
  icl_core::logging::setLogLevel(icl_core::logging::eLL_TRACE);
  LOGGING_INFO(CanOpen, "---------------------------------" << endl <<
                        "-----Running SDO upload test-----" << endl <<
                        "---------------------------------" << endl);

  CanOpenController my_controller("Dummy");
  boost::shared_ptr<icl_hardware::can::tCanDeviceDummy> can_device;
  can_device = boost::dynamic_pointer_cast<icl_hardware::can::tCanDeviceDummy>(my_controller.getCanDevice());

  // Add one node
  my_controller.addGroup<DS301Group>("testgroup");
  my_controller.addNode<DS301Node>(1, "testgroup");
  DS301Group::Ptr my_group = my_controller.getGroup<DS301Group>("testgroup");
  DS301Node::Ptr node = my_group->getNodes().front();


  uint32_t index = 300;
  uint8_t subindex = 3;

  // This is the data that will be uploaded from the slave to the master.
  std::vector<uint8_t> data(4);
  data[0] = 0x01;
  data[1] = 0x02;
  data[2] = 0x03;
  data[3] = 0x04;

  // create node response
  CanMsg response;
  response.id = ds301::ID_TSDO_MIN; // SDO for first node
  response.dlc = 8;
  response.rtr = 0;
  response.data[0] = SDO::SDO_SEG_RES_INIT_UPLOAD_4BYTE;
  response.data[1] = index & 0xff;
  response.data[2] = index >> 8;
  response.data[3] = subindex & 0xff;
  response.data[4] = data[0];
  response.data[5] = data[1];
  response.data[6] = data[2];
  response.data[7] = data[3];
  can_device->addResponse(response);

  std::vector<uint8_t> uploaded_data;

  // If upload succeeds and the response is valid, this will pass
  try
  {
    BOOST_REQUIRE(node->m_sdo.upload(false, index, subindex, uploaded_data));
  }
  catch (const std::exception& e)
  {
    LOGGING_ERROR (CanOpen, e.what() << endl);
  }

  // did the node upload the correct data?
  BOOST_REQUIRE(data == uploaded_data);

  // parse the sent can message that was created by the upload request
  CanMsg msg = can_device->getLastMessage();
  LOGGING_INFO(CanOpen, hexArrayToString(msg.data, msg.dlc) << endl);
  BOOST_REQUIRE(msg.dlc == 8);
  BOOST_REQUIRE(msg.rtr == 0);
  BOOST_REQUIRE(msg.data[0] == SDO::SDO_SEG_REQ_INIT_UPLOAD);
  BOOST_REQUIRE(msg.data[1] == (index & 0xff));
  BOOST_REQUIRE(msg.data[2] == (index >> 8));
  BOOST_REQUIRE(msg.data[3] == subindex);

  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // Now let's generate some faulty responses. They should all fail.
  response.dlc = 13; // some arbitrary illegal number
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW (node->m_sdo.download(false, index, subindex, uploaded_data), TimeoutException);
  response.dlc = 8;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";


  // This will produce a timeout as the message will never reach the SDO
  response.id = ds301::ID_TSDO_MIN-1;
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW (node->m_sdo.download(false, index, subindex, uploaded_data), TimeoutException);
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // This will also produce a timout, as the node to this sdo does not exist
  response.id = ds301::ID_TSDO_MIN+1;
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW (node->m_sdo.download(false, index, subindex, uploaded_data), TimeoutException);
  response.id = ds301::ID_TSDO_MIN;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // This will produce a wrong response error
  response.data[0] = SDO::SDO_SEG_REQ_INIT_UPLOAD | 0x04;
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW (node->m_sdo.download(false, index, subindex, uploaded_data), ResponseException);
  response.data[0] = SDO::SDO_SEG_REQ_INIT_UPLOAD;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // LSB byte of index is wrong
  response.data[1] = (index+1) & 0xff;
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW (node->m_sdo.download(false, index, subindex, uploaded_data), ResponseException);
  response.data[1] = index & 0xff;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // MSB byte of index is wrong
  response.data[2] = (index+256) >> 8;
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW (node->m_sdo.download(false, index, subindex, uploaded_data), ResponseException);
  response.data[2] = index >> 8;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // Subindex is wrong
  response.data[3] = (subindex+1) & 0xff;
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW (node->m_sdo.download(false, index, subindex, uploaded_data), ResponseException);
  response.data[3] = subindex & 0xff;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  // blocked uploads are not yet supported
  can_device->addResponse(response);
  BOOST_REQUIRE(!(node->m_sdo.upload(true, index, subindex, uploaded_data)));

  boost::this_thread::sleep(boost::posix_time::milliseconds(200));
  std::cout << "\n";


  // send a response with an error
  response.data[0] = 0x80; // abort transfer
  std::vector<uint8_t> char_vec = convertToCharVector(0x06010000); // Unsupported access to an object
  response.data[4] = char_vec[0];
  response.data[5] = char_vec[1];
  response.data[6] = char_vec[2];
  response.data[7] = char_vec[3];
  can_device->addResponse(response);
  BOOST_REQUIRE_THROW (node->m_sdo.download(false, index, subindex, uploaded_data), ProtocolException);
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";
}

BOOST_AUTO_TEST_SUITE_END()
