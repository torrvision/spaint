##############################
# CMakeLists.txt for scratch #
##############################

ADD_SUBDIRECTORY(test-boost)

IF(WITH_CUDA)
  ADD_SUBDIRECTORY(test-cuda)
ENDIF()

ADD_SUBDIRECTORY(test-infinitam)

IF(WITH_OPENCV)
  ADD_SUBDIRECTORY(test-opencv)
ENDIF()

IF(WITH_OVR)
  ADD_SUBDIRECTORY(test-ovr)
ENDIF()

ADD_SUBDIRECTORY(test-rafl)
ADD_SUBDIRECTORY(test-sdl)
ADD_SUBDIRECTORY(test-tvgutil)
