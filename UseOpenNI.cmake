###################
# UseOpenNI.cmake #
###################

OPTION(WITH_OPENNI "Build with OpenNI support?" OFF)

IF(WITH_OPENNI)
  SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake-modules/")
  SET(OPEN_NI_ROOT "C:/Program Files/OpenNI2" CACHE FILEPATH "The OpenNI 2 root directory")
  FIND_PACKAGE(OpenNI REQUIRED)
  INCLUDE_DIRECTORIES(${OpenNI_INCLUDE_DIR})
  ADD_DEFINITIONS(-DWITH_OPENNI)
ENDIF()
