####################################
# CMakeLists.txt for tests/scratch #
####################################

ADD_SUBDIRECTORY(boost)

IF(WITH_ARRAYFIRE)
  ADD_SUBDIRECTORY(arrayfire)
ENDIF()

IF(WITH_CUDA)
  ADD_SUBDIRECTORY(cuda)
ENDIF()

ADD_SUBDIRECTORY(eigen)
ADD_SUBDIRECTORY(infermous)
ADD_SUBDIRECTORY(infinitam)

IF(WITH_LEAP)
  ADD_SUBDIRECTORY(leap)
ENDIF()

IF(WITH_OPENCV)
  ADD_SUBDIRECTORY(opencv)
ENDIF()

IF(WITH_OVR)
  ADD_SUBDIRECTORY(ovr)
ENDIF()

ADD_SUBDIRECTORY(rafl)
ADD_SUBDIRECTORY(sdl)
ADD_SUBDIRECTORY(tvgutil)

IF(WITH_VICON)
  ADD_SUBDIRECTORY(vicon)
ENDIF()
