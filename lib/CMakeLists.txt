# Copyright 2011,2012,2016,2018,2019 Free Software Foundation, Inc.
#
# This file was generated by gr_modtool, a tool from the GNU Radio framework
# This file is a part of gr-sic
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

list(APPEND sic_sources
    ble_packet_sink_impl.cc
    ieee802154_packet_sink_impl.cc
    tag_iq_stream_impl.cc
    tagged_iq_to_vector_impl.cc
    transmission_enabler_impl.cc
    periodic_message_source_impl.cc)

set(sic_sources
    "${sic_sources}"
    PARENT_SCOPE)
if(NOT sic_sources)
    message(STATUS "No C++ sources... skipping lib/")
    return()
endif(NOT sic_sources)

add_library(gnuradio-sic SHARED ${sic_sources})
target_link_libraries(gnuradio-sic gnuradio::gnuradio-runtime)
target_include_directories(
    gnuradio-sic
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    PUBLIC $<INSTALL_INTERFACE:include>)
set_target_properties(gnuradio-sic PROPERTIES DEFINE_SYMBOL "gnuradio_sic_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-sic PROPERTIES INSTALL_NAME_DIR
                                                    "${CMAKE_INSTALL_PREFIX}/lib")
endif(APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
gr_library_foo(gnuradio-sic)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

# If your unit tests require special include paths, add them here
#include_directories()
# List all files that contain Boost.UTF unit tests here
list(APPEND test_sic_sources)
# Anything we need to link to for the unit tests go here
list(APPEND GR_TEST_TARGET_DEPS gnuradio-sic)

if(NOT test_sic_sources)
    message(STATUS "No C++ unit tests... skipping")
    return()
endif(NOT test_sic_sources)

foreach(qa_file ${test_sic_sources})
    gr_add_cpp_test("sic_${qa_file}" ${CMAKE_CURRENT_SOURCE_DIR}/${qa_file})
endforeach(qa_file)
