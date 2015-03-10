######################
# UseArrayFire.cmake #
######################

OPTION(WITH_ARRAYFIRE "Build with ArrayFire support?" OFF)

IF(WITH_ARRAYFIRE)

  IF(MSVC_IDE)
  ELSEIF(APPLE)
  ELSEIF("${CMAKE_SYSTEM}" MATCHES "Linux")
    SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/FindArrayFire")
    FIND_PACKAGE(ArrayFire)
  ENDIF()

  INCLUDE_DIRECTORIES(SYSTEM ${ArrayFire_INCLUDE_DIRS})
  ADD_DEFINITIONS(-DWITH_ARRAYFIRE)
ENDIF()
