######################
# UseArrayFire.cmake #
######################

OPTION(WITH_ARRAYFIRE "Build with ArrayFire support?" OFF)

IF(WITH_ARRAYFIRE)
  FIND_PACKAGE(ArrayFire REQUIRED HINTS "${PROJECT_SOURCE_DIR}/libraries/arrayfire-7197760fc82/build")
  INCLUDE_DIRECTORIES(SYSTEM ${ArrayFire_INCLUDE_DIRS})
  ADD_DEFINITIONS(-DWITH_ARRAYFIRE)
ENDIF()
