##################################
# CMakeLists.txt for spaint/apps #
##################################

IF(BUILD_ALL_APPS AND BUILD_EVALUATION_MODULES)
  ADD_SUBDIRECTORY(raflperf)

  IF(WITH_OPENCV)
    ADD_SUBDIRECTORY(raflvis)
    ADD_SUBDIRECTORY(touchtrain)
  ENDIF()
ENDIF()

ADD_SUBDIRECTORY(spaintgui)
