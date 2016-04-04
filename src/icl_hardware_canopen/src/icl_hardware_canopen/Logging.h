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
 * \author  Felix Mauch
 * \date    2015-10-01
 *
 */
//----------------------------------------------------------------------
#ifndef _icl_hardware_canopen_Logging_h_
#define _icl_hardware_canopen_Logging_h_

#include <icl_core_logging/Logging.h>

namespace icl_hardware {
namespace canopen_schunk {

DECLARE_LOG_STREAM(CanOpen);
using icl_core::logging::endl;
using icl_core::logging::flush;

}
}

#endif
