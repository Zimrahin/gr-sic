find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_BLE gnuradio-ble)

FIND_PATH(
    GR_BLE_INCLUDE_DIRS
    NAMES gnuradio/ble/api.h
    HINTS $ENV{BLE_DIR}/include
        ${PC_BLE_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_BLE_LIBRARIES
    NAMES gnuradio-ble
    HINTS $ENV{BLE_DIR}/lib
        ${PC_BLE_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-bleTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_BLE DEFAULT_MSG GR_BLE_LIBRARIES GR_BLE_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_BLE_LIBRARIES GR_BLE_INCLUDE_DIRS)
