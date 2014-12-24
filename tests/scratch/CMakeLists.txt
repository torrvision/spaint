####################################
# CMakeLists.txt for tests/scratch #
####################################

ADD_SUBDIRECTORY(boost)

IF(WITH_CUDA)
  ADD_SUBDIRECTORY(cuda)
ENDIF()

ADD_SUBDIRECTORY(eigen)
ADD_SUBDIRECTORY(infermous)
ADD_SUBDIRECTORY(infinitam)

IF(WITH_OPENCV)
  ADD_SUBDIRECTORY(opencv)
ENDIF()

IF(WITH_OVR)
  ADD_SUBDIRECTORY(ovr)
ENDIF()

ADD_SUBDIRECTORY(rafl)
ADD_SUBDIRECTORY(sdl)
ADD_SUBDIRECTORY(spaint)
ADD_SUBDIRECTORY(tvgutil)
