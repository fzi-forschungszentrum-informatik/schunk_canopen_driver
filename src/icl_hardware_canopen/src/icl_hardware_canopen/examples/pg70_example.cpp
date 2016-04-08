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
#include <icl_hardware_canopen/DS402Group.h>
#include <icl_hardware_canopen/Logging.h>
#include <icl_hardware_canopen/ds402.h>
#include <icl_hardware_canopen/SchunkPowerBallNode.h>

using namespace icl_hardware::canopen_schunk;
int main (int argc, char* argv[])
{
  // Initializing
  icl_core::logging::initialize(argc, argv);

  CanOpenController my_controller("auto");

  DS402Group::Ptr arm_group = my_controller.getGroup<DS402Group>();
  my_controller.addNode<DS402Node>(12);

  DS402Node::Ptr node = my_controller.getNode<DS402Node>(12);

  node->setDefaultPDOMapping(DS402Node::PDO_MAPPING_PROFILE_POSITION_MODE);
  my_controller.initNodes();

  node->printSupportedModesOfOperation();

  ds402::ProfilePositionModeConfiguration config;
  config.profile_acceleration = 1.0;
  config.profile_velocity = 0.5;
  config.use_relative_targets = false;
  config.change_set_immediately = true;
  config.use_blending = false;


  node->setTransmissionFactor(65000);
  node->setModeOfOperation(ds402::MOO_PROFILE_POSITION_MODE);
  node->setDefaultPDOMapping(DS402Node::PDO_MAPPING_PROFILE_POSITION_MODE);

  float current_position;
  float target_position = 0.2;

  my_controller.syncAll();


  node->setupProfilePositionMode(config);
  sleep(1);
  node->enableNode(ds402::MOO_PROFILE_POSITION_MODE);
  sleep(1);

  std::string target_string = "0.0";

  while (target_string != "q")
  {
    LOGGING_INFO(CanOpen, "Please insert a new target position (between 0 and 1, insert q to abort: " << endl);
    std::cin >> target_string;
    std::cout << std::endl;

    try
    {
      target_position = boost::lexical_cast<float>(target_string);
    }
    catch(boost::bad_lexical_cast &)
    {
      if (target_string != "q")
      {
        LOGGING_ERROR (CanOpen, "Invalid input! Please insert a numeric target or q to exit." << endl);
      }
      continue;
    }

    node->setTarget(target_position);

    bool is_reached = false;

    size_t counter = 0;

    while ((!is_reached && counter > 5) || counter <= 5)
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
          try
          {
            node->printStatus();
            is_reached = node->isTargetReached();
            current_position = node->getTPDOValue<int32_t>("measured_position");
            LOGGING_INFO (CanOpen, "Current position: " << current_position << endl);
          }
          catch (const std::exception& e)
          {
            LOGGING_ERROR (CanOpen, e.what() << endl);
            return -1;
          }
        }
      }

      if (counter == 5)
      {
        my_controller.enablePPMotion();
      }
      ++counter;
    }

    LOGGING_INFO (CanOpen, "Target reached" << endl);
  }

  node->disableNode();

  return 0;
}
