# this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# -- END LICENSE BLOCK ------------------------------------------------

#----------------------------------------------------------------------
#file
#
# \author  Felix Mauch <mauch@fzi.de>
# \date    2015-10-7
#
#----------------------------------------------------------------------

#### CanOpen test suite ####
ICMAKER_SET("icl_hardware_canopen_test") # IDE_FOLDER ${ICL_HARDWARE_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  testing_main.cpp
  test_sdo.cpp
  test_emcy.cpp
)

ICMAKER_DEPENDENCIES(
  icl_core
  icl_core_config
  icl_core_logging
  icl_hardware_canopen
)

IF(Boost_FOUND)
  IF(BUILD_SHARED_LIBS)
    ICMAKER_LOCAL_CPPDEFINES("-DBOOST_TEST_DYN_LINK")
  ENDIF(BUILD_SHARED_LIBS)
ENDIF(Boost_FOUND)
ICMAKER_EXTERNAL_DEPENDENCIES(
  Boost_UNIT_TEST_FRAMEWORK
)

ICMAKER_BUILD_TEST()
