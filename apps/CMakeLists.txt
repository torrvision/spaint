##################################
# CMakeLists.txt for spaint/apps #
##################################

IF(BUILD_AUXILIARY_APPS AND BUILD_EVALUATION_MODULES)
  ADD_SUBDIRECTORY(raflperf)
  ADD_SUBDIRECTORY(relocperf)

  IF(WITH_OPENCV)
    ADD_SUBDIRECTORY(raflvis)
    ADD_SUBDIRECTORY(relocvis)

    IF(BUILD_SPAINT AND WITH_ARRAYFIRE)
      ADD_SUBDIRECTORY(touchtrain)
    ENDIF()
  ENDIF()
ENDIF()

IF(BUILD_SCOREFOREST_CONVERTER)
  ADD_SUBDIRECTORY(relocconverter)
ENDIF()

IF(BUILD_SPAINT)
  ADD_SUBDIRECTORY(spaintgui)
ENDIF()
