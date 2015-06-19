##################################
# CMakeLists.txt for spaint/apps #
##################################

IF(BUILD_EVALUATION)
  ADD_SUBDIRECTORY(raflperf)

  IF(WITH_OPENCV)
    ADD_SUBDIRECTORY(raflvis)
  ENDIF()
ENDIF()

ADD_SUBDIRECTORY(spaintgui)
