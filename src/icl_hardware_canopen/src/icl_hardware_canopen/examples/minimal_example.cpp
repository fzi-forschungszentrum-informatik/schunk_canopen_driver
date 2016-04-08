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
  my_controller.addNode<SchunkPowerBallNode>(3);
  my_controller.addNode<SchunkPowerBallNode>(4);
  my_controller.addNode<SchunkPowerBallNode>(5);
  my_controller.addNode<SchunkPowerBallNode>(6);
  my_controller.addNode<SchunkPowerBallNode>(7);
  my_controller.addNode<SchunkPowerBallNode>(8);


  arm_group->initNodes();
  uint8_t nr = 8;
  SchunkPowerBallNode::Ptr node = my_controller.getNode<SchunkPowerBallNode>(nr);

  ds402::ProfilePositionModeConfiguration config;
  config.profile_acceleration = 0.2;
  config.profile_velocity = 0.2;
  config.use_relative_targets = false;
  config.change_set_immediately = true;
  config.use_blending = true;

  arm_group->setupProfilePositionMode(config);

  std::vector<float> targets (6, 0.0);


  arm_group->enableNodes();
  sleep(1);
  arm_group->setTarget(targets);
  sleep(1);
//   node->setTarget(0.3-targets[0]);

  std::vector<bool> foo;

  while (true)
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

//     arm_group->printStatus(8);

    if (arm_group->isTargetReached(foo))
    {
      LOGGING_INFO (CanOpen, "All targets reached" << endl;);
      break;
    }
  }

  arm_group->disableNodes();

  return 0;
}
