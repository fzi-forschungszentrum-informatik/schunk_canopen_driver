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
 * \date    2015-10-8
 *
 */
//----------------------------------------------------------------------

#include <icl_hardware_can/tCanDeviceDummy.h>

#include <icl_hardware_canopen/CanOpenController.h>
#include <icl_hardware_canopen/DS301Group.h>
#include <icl_hardware_canopen/SDO.h>

#include <icl_hardware_canopen/Logging.h>

#include <boost/test/included/unit_test.hpp>


using namespace icl_hardware::canopen_schunk;
BOOST_AUTO_TEST_SUITE( ts_emcy )

/*
 * This test tests the basic EMCY functionality. One example message is sent and compared to what
 * arrives at the node.
 * Additionally, some artificially false data is tested for error handling.
 */
BOOST_AUTO_TEST_CASE( message_parsing )
{
  // Initializing
  icl_core::logging::initialize();
  icl_core::logging::setLogLevel(icl_core::logging::eLL_TRACE);
  LOGGING_INFO(CanOpen, "---------------------------" << endl <<
                        "-----Running EMCY test-----" << endl <<
                        "---------------------------" << endl);

  CanOpenController my_controller("Dummy");
  boost::shared_ptr<icl_hardware::can::tCanDeviceDummy> can_device;
  can_device = boost::dynamic_pointer_cast<icl_hardware::can::tCanDeviceDummy>(my_controller.getCanDevice());

  // Add one node
  my_controller.addGroup<DS301Group>("testgroup");
  my_controller.addNode<DS301Node>(1, "testgroup");
  DS301Group::Ptr my_group = my_controller.getGroup<DS301Group>("testgroup");
  DS301Node::Ptr node = my_group->getNodes().front();

  // create can message
  CanMsg msg;
  msg.id = ds301::ID_EMCY_MIN; // EMCY for first node
  msg.dlc = 8;
  msg.rtr = 0;
  // error code 0x8130
  msg.data[0] = 0x30;
  msg.data[1] = 0x81;
  // Error registers 0x01, 0x02, 0x03, 0x20
  msg.data[2] = 0x27;

  // some arbitrary manufacturer-specific data
  msg.data[3] = 0x03;
  msg.data[4] = 0x04;
  msg.data[5] = 0x05;
  msg.data[6] = 0x06;
  msg.data[7] = 0x07;
  can_device->addResponse(msg, false);

  // wait for the response being processed
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));

  uint16_t eec_ret;
  uint8_t error_register_ret;
  std::vector<uint8_t> msef_ret;
  BOOST_REQUIRE( node->m_emcy->getErrorInformation(eec_ret, error_register_ret, msef_ret));

  BOOST_REQUIRE( eec_ret == msg.data[0] + (msg.data[1] << 8) );
  BOOST_REQUIRE( error_register_ret == msg.data[2]);
  BOOST_REQUIRE( msef_ret[0] == msg.data[3]);
  BOOST_REQUIRE( msef_ret[1] == msg.data[4]);
  BOOST_REQUIRE( msef_ret[2] == msg.data[5]);
  BOOST_REQUIRE( msef_ret[3] == msg.data[6]);
  BOOST_REQUIRE( msef_ret[4] == msg.data[7]);
  BOOST_REQUIRE( node->m_emcy->getEmcyStatus() == EMCY::EMCY_STATE_ERROR_OCCURED);

  // send a ERROR_FREE_EMCY
  msg.data[0] = 0x00;
  msg.data[1] = 0x00;

  can_device->addResponse(msg, false);

  // wait for the response being processed
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));

  // Now there should be no error
  BOOST_REQUIRE( !(node->m_emcy->getErrorInformation(eec_ret, error_register_ret, msef_ret)));
  BOOST_REQUIRE( node->m_emcy->getEmcyStatus() == EMCY::EMCY_STATE_ERROR_FREE);
  std::cout << "\n";

  // WARNING: Do not remove the following 4 lines as they reset the error state to non-error. Otherwise the following tests will fail!
  // send a ERROR_FREE_EMCY
  msg.data[0] = 0x00;
  msg.data[1] = 0x00;
  can_device->addResponse(msg, false);
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));

  msg.dlc = 13; // some arbitrary illegal number
  can_device->addResponse(msg, false);
  BOOST_REQUIRE( !(node->m_emcy->getErrorInformation(eec_ret, error_register_ret, msef_ret)));
  BOOST_REQUIRE( node->m_emcy->getEmcyStatus() == EMCY::EMCY_STATE_ERROR_FREE);
  msg.dlc = 8;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

  msg.id = ds301::ID_EMCY_MIN + 1; // This node id does not exist
  can_device->addResponse(msg, false);
  BOOST_REQUIRE( !(node->m_emcy->getErrorInformation(eec_ret, error_register_ret, msef_ret)));
  BOOST_REQUIRE( node->m_emcy->getEmcyStatus() == EMCY::EMCY_STATE_ERROR_FREE);
  msg.id = ds301::ID_EMCY_MIN;
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  std::cout << "\n";

}


BOOST_AUTO_TEST_SUITE_END()
