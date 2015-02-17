##################################
# CMakeLists.txt for spaint/apps #
##################################

IF(WITH_OPENCV)
  ADD_SUBDIRECTORY(raflperf)
ENDIF()
ADD_SUBDIRECTORY(raflvis)
ADD_SUBDIRECTORY(spaintgui)
