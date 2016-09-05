##################################
# CMakeLists.txt for spaint/apps #
##################################

IF(BUILD_AUXILIARY_APPS AND BUILD_EVALUATION_MODULES)
  ADD_SUBDIRECTORY(raflperf)

  IF(WITH_OPENCV)
    ADD_SUBDIRECTORY(raflvis)

    IF(BUILD_SPAINT AND WITH_ARRAYFIRE)
      ADD_SUBDIRECTORY(touchtrain)
    ENDIF()
  ENDIF()
ENDIF()

IF(BUILD_SPAINT)
  ADD_SUBDIRECTORY(spaintgui)
ENDIF()
