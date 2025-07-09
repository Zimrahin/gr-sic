# -*- coding: utf-8 -*-
#
# Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
# Copyright 2025 Inria.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Include python install macros
########################################################################
include(GrPython)
if(NOT PYTHONINTERP_FOUND)
    return()
endif()

########################################################################
# Install python sources
########################################################################
gr_python_install(
    FILES
    __init__.py
    transmitters.py
    filters.py
    modulation.py
    packet_utils.py
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/sic/utils
)

