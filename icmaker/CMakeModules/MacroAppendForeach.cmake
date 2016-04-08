# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This file is part of the icmaker build system.
#
# This program is free software licensed under the BSD License. You can
# find a copy of this license in the LICENSE folder in the top directory
# of the source code.
#
# © Copyright 2016 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------

# - MACRO_APPEND_FOREACH(<_targetlist> _prefix _suffix <_sourcelist> )

# Copyright (c) 2009, Christian Stimming

# Appends each element of the <_sourcelist> to the <_targetlist>, but
# with the _prefix prepended and _suffix appended. Note: If no suffix
# is desired, pass an empty string ("") there.

# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

MACRO (MACRO_APPEND_FOREACH _target _prefix _suffix)

  FOREACH (_loop_var ${ARGN})

	SET (${_target} ${${_target}} "${_prefix}${_loop_var}${_suffix}")
	
  ENDFOREACH (_loop_var)

ENDMACRO (MACRO_APPEND_FOREACH)
