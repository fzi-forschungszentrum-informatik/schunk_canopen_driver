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

#include <icl_hardware_canopen/CanOpenController.h>
#include <icl_hardware_canopen/DS301Group.h>
#include <icl_hardware_canopen/SchunkPowerBallNode.h>
#include <icl_hardware_canopen/DS402Group.h>
#include <icl_hardware_canopen/exceptions.h>
#include <icl_hardware_canopen/PDO.h>

#include <icl_hardware_canopen/Logging.h>
#include "../ds402.h"

#include <cstdlib> // getenv
#include <boost/filesystem.hpp>

using namespace icl_hardware::canopen_schunk;

int main (int argc, char* argv[])
{
  // Initializing
  icl_core::logging::initialize(argc, argv);

  CanOpenController my_controller("auto");

  my_controller.addGroup<DS402Group>("arm");
  my_controller.addNode<SchunkPowerBallNode>(3, "arm");
  my_controller.addNode<SchunkPowerBallNode>(4, "arm");
  my_controller.addNode<SchunkPowerBallNode>(5, "arm");
  my_controller.addNode<SchunkPowerBallNode>(6, "arm");
  my_controller.addNode<SchunkPowerBallNode>(7, "arm");
  my_controller.addNode<SchunkPowerBallNode>(8, "arm");

  std::vector <uint8_t> output_data;
  sleep(5); // wait for 5 seconds

  boost::filesystem::path resources_path;
  char const* tmp = std::getenv("CANOPEN_RESOURCE_PATH");
  if (tmp == NULL)
  {
    LOGGING_WARNING_C(
        CanOpen,
        CanOpenController,
        "The environment variable 'CANOPEN_RESOURCE_PATH' could not be read. Using relative path 'resources/'" << endl);
    resources_path = boost::filesystem::path("resources");
  }
  else
  {
    resources_path = boost::filesystem::path(tmp);
  }

  std::string emcy_emergency_errors_filename = (resources_path / boost::filesystem::path("EMCY_schunk.ini")).string();
  EMCY::addEmergencyErrorMap( emcy_emergency_errors_filename, "schunk_error_codes");



  SchunkPowerBallNode::Ptr node_3 = my_controller.getNode<SchunkPowerBallNode> (3);
  SchunkPowerBallNode::Ptr node_4 = my_controller.getNode<SchunkPowerBallNode> (4);
  SchunkPowerBallNode::Ptr node_5 = my_controller.getNode<SchunkPowerBallNode> (5);
  SchunkPowerBallNode::Ptr node_6 = my_controller.getNode<SchunkPowerBallNode> (6);
  SchunkPowerBallNode::Ptr node_7 = my_controller.getNode<SchunkPowerBallNode> (7);
  SchunkPowerBallNode::Ptr node_8 = my_controller.getNode<SchunkPowerBallNode> (8);


  DS402Group::Ptr arm_group = my_controller.getGroup<DS402Group>("arm");

  SchunkPowerBallNode::Ptr node = node_7;
//   SchunkPowerBallNode::Ptr node;
  if (node)
  {
    node->m_nmt.preOperational();
    node->printPDOMapping();
//     PDO::MappingConfigurationList rpdo_mappings;
//     // Map control and status word to the first PDO (receive and transmit respectively)
//     rpdo_mappings.push_back(PDO::MappingConfiguration(0x6040, 0, 16, "control_word"));
//     rpdo_mappings.push_back(PDO::MappingConfiguration(0x607a, 0, 32, "target_position"));
//
//     node->appendPDOMappingSingle(rpdo_mappings, 1, PDO::SYNCHRONOUS_CYCLIC, DS301Node::RECEIVE_PDO, false);
    node->initNode();
    node_8->initNode();
    node->printSupportedModesOfOperation();

    node->setModeOfOperation(ds402::MOO_PROFILE_POSITION_MODE);

//     ds402::ProfilePositionModeConfiguration config;
//     config.profile_acceleration = 0.2;
//     config.profile_velocity = 0.2;
//     config.use_relative_targets = false;
//     config.change_set_immediately = true;
//     config.use_blending = true;
//     config.use_relative_targets = false;
//
//     node_8->setupProfilePositionMode(config);


//     uint32_t data32;
//     node->m_sdo.upload(false, 0x1600, 0x01, data32);
//     LOGGING_INFO(CanOpen, "RPDO-mapping 1: " << hexToString(data32) << endl);
//     node->m_sdo.upload(false, 0x1600, 0x02, data32);
//     LOGGING_INFO(CanOpen, "RPDO-mapping 2: " << hexToString(data32) << endl);
//     node->m_sdo.upload(false, 0x1600, 0x03, data32);
//     LOGGING_INFO(CanOpen, "RPDO-mapping 3: " << hexToString(data32) << endl);
//
//     node->m_sdo.upload(false, 0x1a00, 0x01, data32);
//     LOGGING_INFO(CanOpen, "TPDO-mapping 1: " << hexToString(data32) << endl);
//     node->m_sdo.upload(false, 0x1a00, 0x02, data32);
//     LOGGING_INFO(CanOpen, "TPDO-mapping 2: " << hexToString(data32) << endl);
//     node->m_sdo.upload(false, 0x1a00, 0x03, data32);
//     LOGGING_INFO(CanOpen, "TPDO-mapping 3: " << hexToString(data32) << endl);
//     node->configureHomingMethod(33);
  }
  else
  {
    arm_group->initNodes();
  }


//   sleep (1);
  std::cout << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;

  LOGGING_DEBUG (CanOpen, "Running syncAll once" << endl);
  my_controller.syncAll();


//   sleep(3);

//   node_8->printStatus();

  try
  {
    if (node)
    {
//       node->initDS402State(ds402::STATE_OPERATION_ENABLE);
//       node->home();
    }
    else
    {
      node_8->commutationSearch();
      std::cout << std::endl << std::endl << std::endl << std::endl << std::endl;
      sleep(2);
      node_7->commutationSearch();
      sleep(2);
      node_6->commutationSearch();
      sleep(2);
      node_5->commutationSearch();
      sleep(2);
      node_4->commutationSearch();
      sleep(2);
      node_3->commutationSearch();
//       arm_group->initDS402State(ds402::STATE_OPERATION_ENABLE);
//       std::vector<DS301Node::Ptr> nodes = arm_group->
    }
  }
  catch (const std::exception& e)
  {
    LOGGING_ERROR (CanOpen, e.what() << endl);
    return -1;
  }

  // check for recorded errors
//   node_3->m_emcy->printLastErrors(node_3->m_sdo);
//   node_3->m_emcy->clearErrorHistory(node_3->m_sdo);
//   node_3->m_emcy->printLastErrors(node_3->m_sdo);


  std::cout << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;

//   return 0;

  size_t counter = 0;
  double target_position = 0.05;
  double current_position = 0;

  ////////////  state machine test   ////////////
//   while (true)
//   {
//     try {
//       my_controller.syncAll();
//     }
//     catch (const std::exception& e)
//     {
//       LOGGING_ERROR (CanOpen, e.what() << endl);
//       return  -1;
//     }
//
//     usleep(10000);
//
//     int state_int;
//     std::cout << "In which state should I go? ";
//     std::cin >> state_int;
//     std::cout << std::endl;
//
//     ds402::eState state = static_cast<ds402::eState>(state_int);
//
//     node->initDS402State(state);
//
//     node->printStatus();
//
//     ++counter;
//   }

  ////////////  IPM test   ////////////
//   node->home();

  node_8->enableNode(ds402::MOO_PROFILE_POSITION_MODE);
  while (counter < 5000)
  {
    try {
      my_controller.syncAll();
    }
    catch (const std::exception& e)
    {
      LOGGING_ERROR (CanOpen, e.what() << endl);
      return  -1;
    }

    usleep(10000);

    if (counter > 2)
    {
      if (node)
      {
        node->printStatus();
      }
      else
      {
        arm_group->printStatus();
      }
    }

    if (counter == 5)
    {
      try
      {
        node->enableNode(ds402::MOO_INTERPOLATED_POSITION_MODE);
        current_position = static_cast<double>(node->getTPDOValue<int32_t>("measured_position")) / SchunkPowerBallNode::RAD_TO_STEPS_FACTOR;
        LOGGING_INFO (CanOpen, "Current position: " << current_position << endl);
      }
      catch (const std::exception& e)
      {
        LOGGING_ERROR (CanOpen, e.what() << endl);
        return -1;
      }
    }

    if (counter == 50 )
    {
      node_8->setTarget(1.57);
    }

    if (counter == 500 )
    {
      node_8->setTarget(-1.57);
    }

    if (counter == 1000 )
    {
      node_8->setTarget(1.57);
    }
    if (counter == 2000 )
    {
      node_8->setTarget(-1.57);
    }

    if (counter > 5 && counter < 299)
    {
      double starting_point = asin(current_position);
      target_position = sin(0.01*(counter-6) + starting_point);
      LOGGING_INFO (CanOpen, "Sinus target: " << target_position << endl);
//       return 0;
      node->setTarget(target_position);
    }

//     if (counter == 24)
//     {
//       try
//       {
// //         uint32_t vel = 1;
// //         node->m_sdo.download(false, 0x60FF, 0, vel);
// //         node->setTarget(vel);
// //         node_8->resetFault();
// //         node->setTarget(13);
//         int32_t data32;
//         uint32_t udata32;
//
//         // Buffer size
//         node->m_sdo.upload(false, 0x60c4, 2, udata32);
//         LOGGING_INFO(CanOpen, "Number of buffer entries: " << static_cast<int>(udata32) << endl);
//
//         // interpolation data record
//         for (size_t i=1; i <= udata32; ++i)
//         {
//           node->m_sdo.upload(false, 0x60c1, i, data32);
//           LOGGING_INFO(CanOpen, "Interpolation data record " << static_cast<int>(i) << ": " << static_cast<int>(data32) << endl);
//         }
//       }
//       catch (const std::exception& e)
//       {
//         LOGGING_ERROR (CanOpen, e.what() << endl);
//         return -1;
//       }
//     }
    if (counter == 300)
    {
      node->quickStop();
      LOGGING_INFO (CanOpen, "QuickStopped" << endl);
    }

    if (counter == 600)
    {
      current_position = static_cast<double>(node->getTPDOValue<int32_t>("measured_position")) / SchunkPowerBallNode::RAD_TO_STEPS_FACTOR;
      LOGGING_INFO (CanOpen, "Current position: " << current_position << endl);
      node->enableNode(ds402::MOO_INTERPOLATED_POSITION_MODE);
    }

    if (counter >= 605 && counter < 800 )
    {
      double starting_point = asin(current_position);
      target_position = sin(0.01*(counter-605) + starting_point);
      LOGGING_INFO (CanOpen, "Sinus target: " << target_position << endl);
      node->setTarget(target_position);
//       return 0;
    }

    if (counter == 600)
    {
      try
      {
//         node_3->initDS402State (icl_hardware::canopen_schunk::ds402::STATE_SWITCHED_ON);
//         node_4->initDS402State (icl_hardware::canopen_schunk::ds402::STATE_SWITCHED_ON);
//         node_5->initDS402State (icl_hardware::canopen_schunk::ds402::STATE_SWITCHED_ON);
//         node_6->initDS402State (icl_hardware::canopen_schunk::ds402::STATE_SWITCHED_ON);
//         node_7->initDS402State (icl_hardware::canopen_schunk::ds402::STATE_SWITCHED_ON);
//         node_8->initDS402State (icl_hardware::canopen_schunk::ds402::STATE_SWITCHED_ON);
      }
      catch (const std::exception& e)
      {
        LOGGING_ERROR (CanOpen, e.what() << endl);
        return -1;
      }

    }

    if (counter == 880)
    {
      node->closeBrakes();
    }


    if (counter == 1000)
    {
      current_position = static_cast<double>(node->getTPDOValue<int32_t>("measured_position")) / SchunkPowerBallNode::RAD_TO_STEPS_FACTOR;
      LOGGING_INFO (CanOpen, "Current position: " << current_position << endl);
      node->openBrakes();
    }

    if (counter >= 1005 && counter < 2000)
    {
      LOGGING_INFO (CanOpen, "Current position: " << current_position << endl);
      double starting_point = asin(current_position);
      target_position = sin(0.01*(counter-1005) + starting_point);
      LOGGING_INFO (CanOpen, "Sinus target: " << target_position << endl);
      node->setTarget(target_position);
    }

    if (counter == 2000)
    {
      current_position = static_cast<double>(node->getTPDOValue<int32_t>("measured_position")) / SchunkPowerBallNode::RAD_TO_STEPS_FACTOR;
      LOGGING_INFO (CanOpen, "Current position: " << current_position << endl);
      node->enableNode(ds402::MOO_INTERPOLATED_POSITION_MODE);
    }

    if (counter >= 2005)
    {
      double starting_point = asin(current_position);
      target_position = sin(0.01*(counter-2005) + starting_point);
      LOGGING_INFO (CanOpen, "Sinus target: " << target_position << endl);
      node->setTarget(target_position);
    }

    if (counter == 48)
    {
//       node->disableNode();
    }
    ++counter;
  }

  //////////  PPM test   ////////////


//   ds402::Controlword word;
//   word.all = 0x2f;
//   target_position = 0.5;
//
//   // enable node
//   node->enableNode(ds402::MOO_PROFILE_POSITION_MODE);
//   node_7->enableNode(ds402::MOO_PROFILE_POSITION_MODE);
//   // set target
// //   node->m_sdo.download<uint32_t>(false, 0x607a, 0, 0);
// //   sleep(1);
// //   my_controller.syncAll();
// //   node->printStatus();
//
// //   return 0;
//   while (true)
//   {
//     try {
//       my_controller.syncAll();
//     }
//     catch (const std::exception& e)
//     {
//       LOGGING_ERROR (CanOpen, e.what() << endl);
//       return  -1;
//     }
//
//     usleep(10000);
//
//     int16_t profile;
//     node->printStatus();
//     node->m_sdo.upload<int16_t>(false, 0x6086, 0, profile);
// //     LOGGING_INFO (CanOpen, "Motion type profile is " << profile << endl);
//
//     if (counter == 5 )
//     {
//       LOGGING_INFO (CanOpen, "Publishing target position to device: " << target_position << endl);
//       node->setTarget(target_position);
//       node_7->setTarget(-target_position);
//     }
//
//
//     if(counter == 100)
//     {
//       target_position = 2 * target_position;
//       node->setTarget(target_position);
//       node_7->setTarget(-target_position);
//     }
//
//     if(counter == 400)
//     {
//       target_position = 0;
//       node->setTarget(target_position);
//       node_7->setTarget(target_position);
//     }
//
//     LOGGING_INFO (CanOpen, "Counter: " << counter << endl);
//
//     ++counter;
//   }




  ////////////  dummy loop   ////////////

//   while (true)
//   {
//     try {
//       my_controller.syncAll();
//     }
//     catch (const std::exception& e)
//     {
//       LOGGING_ERROR (CanOpen, e.what() << endl);
//       return  -1;
//     }
//
//     usleep(10000);
//   }

  node->disableNode();
  node_8->disableNode();

  return 0;
}
