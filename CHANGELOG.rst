^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package schunk_canopen_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.7 (2017-10-11)
------------------
* use more advanced pcan-device auto mode
* Contributors: Felix Mauch

1.0.6 (2016-05-17)
------------------
* made xacro usage conform to current standards from jade and kinetic
* implement prepareSwitch instead of canSwitch
* fix: do not start boost thread explicitly
* Contributors: Felix Mauch

1.0.5 (2016-05-12)
------------------
* added missing runtime dependencies
* Contributors: Felix Mauch

1.0.4 (2016-05-11)
------------------
* use the "auto" keyword for the identifier in the node executables
* made the check for the resource path safe to null pointers
* install launch files.
* Contributors: Felix Mauch

1.0.3 (2016-05-04)
------------------
* fix: Dependencies were wrong and others were missing
* Updated and sorted package dependencies
* Corrected maintainer name
* Contributors: Felix Mauch

1.0.2 (2016-05-02)
------------------
* remove icl_hardware_canopen from dependencies as it is provided by this
  package
* Contributors: Felix Mauch

1.0.1 (2016-04-08)
------------------
* First initial release of the schunk_canopen_driver for ROS
* Contributors: Andreas Hermann, Felix Mauch, Georg Heppner, Pascal Becker
