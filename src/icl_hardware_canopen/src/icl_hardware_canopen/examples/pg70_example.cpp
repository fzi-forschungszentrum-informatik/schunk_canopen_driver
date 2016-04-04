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

  CanOpenController my_controller("/dev/pcanusb1");

  DS402Group::Ptr arm_group = my_controller.getGroup<DS402Group>();
  my_controller.addNode<DS402Node>(12);

  DS402Node::Ptr node = my_controller.getNode<DS402Node>(12);

  node->setDefaultPDOMapping(DS402Node::PDO_MAPPING_INTERPOLATED_POSITION_MODE);
  node->initNode();

  node->printSupportedModesOfOperation();

  node->setModeOfOperation(ds402::MOO_INTERPOLATED_POSITION_MODE);
  node->setDefaultPDOMapping(DS402Node::PDO_MAPPING_INTERPOLATED_POSITION_MODE);

  int32_t current_position;
  int32_t target_position;
  size_t counter = 0;

  my_controller.syncAll();

  sleep(1);

  while (counter < 300)
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
        node->enableNode(ds402::MOO_INTERPOLATED_POSITION_MODE);
        node->printStatus();
      }
      else
      {
        arm_group->printStatus();
      }
    }

//     node->m_sdo.upload(false, 0x6064, 0, position);
//     LOGGING_INFO (CanOpen, "Device is at position " << position << endl);
//
    if (counter == 5)
    {
      try
      {
        current_position = node->getTPDOValue<int32_t>("measured_position");
        LOGGING_INFO (CanOpen, "Current position: " << current_position << endl);
      }
      catch (const std::exception& e)
      {
        LOGGING_ERROR (CanOpen, e.what() << endl);
        return -1;
      }
    }

    if (counter > 5 && counter < 299)
    {
      target_position = current_position + 100*(counter-5);
      LOGGING_INFO (CanOpen, "Sinus target: " << target_position << endl);
      node->setTarget(target_position);
    }

    ++counter;
  }

  node->disableNode();

  return 0;
}
