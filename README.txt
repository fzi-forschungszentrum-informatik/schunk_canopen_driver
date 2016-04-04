This is the driver package for the SCHUNK LWA4P and other CanOpen enabled devices.
It was developed on behalf of SCHUNK GmbH, Lauffen/Neckar, Germany
at the FZI Research Center for Information Technology in Karlsruhe, Germany.

The package contains the following core components:
- The independent C++ CanOpen driver library
- The ROS abstraction layer for it
- 3D model and kinematics description for visualization and planning
These main components are distributed under a LGPL license.

Furthermore this package contains a build system that is not
part of the CanOpen Driver but which is needed to build it:
- icmaker  (BSD License)
and depends on two other packages:
-fzi_icl_core  containing the icl_core (LGPL License)
-fzi_icl_can containing the icl_can (LGPL License)

These components were independently developed at the
FZI Research Center for Information Technology in Karlsruhe, Germany.

Please refer to the wiki page for installation and usage.

See license folder for the license texts.
