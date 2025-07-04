# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT sic_sources)
    message(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT sic_sources)

########################################################################
# Check for pygccxml
########################################################################
gr_python_check_module_raw("pygccxml" "import pygccxml" PYGCCXML_FOUND)

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND sic_python_files
    ble_packet_sink_python.cc
    ieee802154_packet_sink_python.cc
    tag_iq_stream_python.cc
    tagged_iq_to_vector_python.cc
    transmission_enabler_python.cc python_bindings.cc)

gr_pybind_make_oot(sic ../../.. gr::sic "${sic_python_files}")

# copy bindings extension for use in QA test module
add_custom_command(
    TARGET sic_python
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:sic_python>
            ${PROJECT_BINARY_DIR}/test_modules/gnuradio/sic/)

install(
    TARGETS sic_python
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/sic
    COMPONENT pythonapi)
