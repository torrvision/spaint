######################
# UseRealSense.cmake #
######################

OPTION(WITH_REALSENSE "Build with Intel RealSense support?" OFF)

IF(WITH_REALSENSE)
  FIND_PATH(RealSense_ROOT librealsense.vc12 HINTS "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM/librealsense")
  FIND_PATH(RealSense_INCLUDE_DIR librealsense HINTS "${RealSense_ROOT}/include")
  FIND_LIBRARY(RealSense_LIBRARY realsense HINTS "${RealSense_ROOT}/bin/x64")

  INCLUDE_DIRECTORIES(${RealSense_INCLUDE_DIR})

  ADD_DEFINITIONS(-DCOMPILE_WITH_RealSense)
  ADD_DEFINITIONS(-DWITH_REALSENSE)
ENDIF()
