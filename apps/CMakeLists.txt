##################################
# CMakeLists.txt for spaint/apps #
##################################

IF(BUILD_EVALUATION)
  ADD_SUBDIRECTORY(raflperf)
  ADD_SUBDIRECTORY(touchtrain)

  IF(WITH_OPENCV)
    ADD_SUBDIRECTORY(raflvis)
  ENDIF()
ENDIF()

ADD_SUBDIRECTORY(spaintgui)
