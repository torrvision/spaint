################
# UseVTK.cmake #
################

OPTION(WITH_VTK "Build with VTK support?" OFF)

IF(WITH_VTK)
  FIND_PACKAGE(VTK REQUIRED)
  INCLUDE(${VTK_USE_FILE})
  ADD_DEFINITIONS(-DWITH_VTK)
ENDIF()
