find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_SIC gnuradio-sic)

FIND_PATH(
    GR_SIC_INCLUDE_DIRS
    NAMES gnuradio/sic/api.h
    HINTS $ENV{SIC_DIR}/include
        ${PC_SIC_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_SIC_LIBRARIES
    NAMES gnuradio-sic
    HINTS $ENV{SIC_DIR}/lib
        ${PC_SIC_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-sicTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_SIC DEFAULT_MSG GR_SIC_LIBRARIES GR_SIC_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_SIC_LIBRARIES GR_SIC_INCLUDE_DIRS)
