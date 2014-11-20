###################################
# CMakeLists.txt for spaint/tests #
###################################

OPTION(BUILD_MIKETESTS "Build Michael's tests?" OFF)

IF(BUILD_MIKETESTS)
  ADD_SUBDIRECTORY(mike)
ENDIF()

OPTION(BUILD_SCRATCHTESTS "Build the scratch tests?" ON)

IF(BUILD_SCRATCHTESTS)
  ADD_SUBDIRECTORY(scratch)
ENDIF()

OPTION(BUILD_UNITTESTS "Build the unit tests?" ON)

IF(BUILD_UNITTESTS)
  ADD_SUBDIRECTORY(unit)
ENDIF()