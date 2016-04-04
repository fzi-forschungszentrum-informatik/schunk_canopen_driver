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
 * \date    2015-10-1
 *
 *
 */
//----------------------------------------------------------------------

#ifndef SYNC_H
#define SYNC_H

#include <icl_hardware_can/tCanDevice.h>
#include "helper.h"
#include "ds301.h"

namespace icl_hardware {
namespace canopen_schunk {

// TODO(optional) include a sync counter that is transmitted, however.. i can not see the point in it DS301 clause 7.5.2.22: "The SYNC consumer shall ignore the value itself"

/*!
 * \brief sendSync sends a SYNC message which is used to ensure synchronous processing of all CanOpen nodes.
 * \param can_device device handle to use for the communication
 */
static inline void sendSync(const CanDevPtr &can_device)
{
  can_device->Send(CanMsg(ds301::ID_SYNC,0,0,reinterpret_cast<unsigned char*>(NULL)));
}

}} // end of NS
#endif // SYNC_H
