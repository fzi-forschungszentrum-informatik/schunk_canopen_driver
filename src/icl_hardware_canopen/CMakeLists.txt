# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

# To be used by other modules
SET(ICL_HARDWARE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE INTERNAL "")
SET(ICL_HARDWARE_IDE_FOLDER "icl_hardware_canopen")


ICMAKER_REGISTER_PACKAGE(icl_hardware_canopen)

ADD_SUBDIRECTORY(src/icl_hardware_canopen)

ICMAKER_CONFIGURE_PACKAGE()
