# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
ICMAKER_SET("icl_hardware_canopen" IDE_FOLDER ${ICL_HARDWARE_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
CanOpenController.cpp
CanOpenReceiveThread.cpp
DS301Group.cpp
DS402Group.cpp
DS301Node.cpp
DS402Node.cpp
SchunkPowerBallNode.cpp
Logging.cpp
SDO.cpp
NMT.cpp
HeartBeatMonitor.cpp
RPDO.cpp
TPDO.cpp
PDO.cpp
EMCY.cpp
helper.cpp
)

ICMAKER_ADD_HEADERS(
CanOpenController.h
CanOpenController.hpp
CanOpenReceiveThread.h
DS301Group.h
DS301Group.hpp
DS402Group.h
DS402Group.hpp
DS301Node.h
DS402Node.h
SchunkPowerBallNode.h
Logging.h
SDO.h
NMT.h
HeartBeatMonitor.h
PDO.h
RPDO.h
TPDO.h
EMCY.h
ds301.h
ds402.h
sync.h
helper.h
exceptions.h
)

ICMAKER_LOCAL_CPPDEFINES(-DICL_HARDWARE_CAN_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_ICL_HARDWARE_CAN_ -DOSNAME_LINUX)
ICMAKER_INCLUDE_DIRECTORIES(${ICL_HARDWARE_INCLUDE_DIRS})

IF (PeakCan_FOUND)
  ICMAKER_EXTERNAL_DEPENDENCIES(EXPORT
    PeakCan
  )
ENDIF ()

ICMAKER_INTERNAL_DEPENDENCIES(
  PeakCan
  icl_core
  icl_core_config
  icl_core_logging
  icl_hardware_can
)

# Websockets are a later expansion that is not considered at the moment but prepared for
ICMAKER_DEPENDENCIES(OPTIONAL EXPORT
 icl_comm_websocket
)

ICMAKER_BUILD_LIBRARY()
ICMAKER_INSTALL_HEADERS(${icmaker_target})

ADD_SUBDIRECTORY(test)
add_subdirectory(examples)
