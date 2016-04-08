# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This file is part of the icmaker build system.
#
# This program is free software licensed under the BSD License. You can
# find a copy of this license in the LICENSE folder in the top directory
# of the source code.
#
# © Copyright 2016 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------

# - Try to find the antlr library (http://www.antlr.org/)
#
# Once done this will define
#
# ANTLR_FOUND - system has libantlr
# ANTLR_INCLUDE_DIR - the libantlr include directory
# ANTLR_LIBRARIES - Link these to use libantlr
#

# Copyright (c) 2009, Patrick Spendrin, <ps_ml@gmx.de>
#
# Redistribution and use is allowed according to the terms of the BSD license, as applying for all kdelibs cmake modules


if (ANTLR_INCLUDE_DIR AND ANTLR_LIBRARIES)

    # in cache already
    set(ANTLR_FOUND TRUE)

else (ANTLR_INCLUDE_DIR AND ANTLR_LIBRARIES)

    find_path(ANTLR_INCLUDE_DIR antlr3.h)

    find_library(ANTLR_LIBRARIES NAMES antlr3 antlr3c libantlr3 libantlr3c)

    if (ANTLR_INCLUDE_DIR AND ANTLR_LIBRARIES)
        set(ANTLR_FOUND TRUE)
        # TODO version check is missing
    endif (ANTLR_INCLUDE_DIR AND ANTLR_LIBRARIES)

    if (ANTLR_FOUND)
        if (NOT ANTLR_FIND_QUIETLY)
            message(STATUS "Found Antlr3 C runtime: ${ANTLR_LIBRARIES}")
        endif (NOT ANTLR_FIND_QUIETLY)
    else (ANTLR_FOUND)
        if (ANTLR_FIND_REQUIRED)
            if (NOT ANTLR_INCLUDE_DIR)
                message(FATAL_ERROR "Could NOT find Antlr3 header files")
            endif (NOT ANTLR_INCLUDE_DIR)
            if (NOT ANTLR_LIBRARIES)
                message(FATAL_ERROR "Could NOT find Antlr3 C runtime library")
            endif (NOT ANTLR_LIBRARIES)
        endif (ANTLR_FIND_REQUIRED)
    endif (ANTLR_FOUND)

    mark_as_advanced(ANTLR_INCLUDE_DIR ANTLR_LIBRARIES)
  
endif (ANTLR_INCLUDE_DIR AND ANTLR_LIBRARIES)
