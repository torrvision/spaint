###################################
# CMakeLists.txt for spaint/tests #
###################################

ADD_SUBDIRECTORY(test-cuda)
ADD_SUBDIRECTORY(test-infinitam)
ADD_SUBDIRECTORY(test-sdl)

IF(WITH_BOOST)
  ADD_SUBDIRECTORY(test-boost)
ENDIF()

IF(WITH_OPENCV)
  ADD_SUBDIRECTORY(test-opencv)
ENDIF()
