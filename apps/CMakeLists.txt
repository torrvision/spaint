##################################
# CMakeLists.txt for spaint/apps #
##################################

ADD_SUBDIRECTORY(raflperf)

IF(WITH_OPENCV)
  ADD_SUBDIRECTORY(raflvis)
ENDIF()

ADD_SUBDIRECTORY(spaintgui)
