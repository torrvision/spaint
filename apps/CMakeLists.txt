##################################
# CMakeLists.txt for spaint/apps #
##################################

IF(BUILD_AUXILIARY_APPS)
  IF(BUILD_EVALUATION_MODULES)
    ADD_SUBDIRECTORY(raflperf)
    ADD_SUBDIRECTORY(relocperf)

    IF(WITH_OPENCV)
      ADD_SUBDIRECTORY(raflvis)

      IF(BUILD_RELOCVIS)
        ADD_SUBDIRECTORY(relocvis)
      ENDIF()

      IF(BUILD_SPAINT AND WITH_ARRAYFIRE)
        ADD_SUBDIRECTORY(touchtrain)
      ENDIF()
    ENDIF()
  ENDIF()

  IF(WITH_SCOREFORESTS)
    ADD_SUBDIRECTORY(relocconverter)
    IF(WITH_OPENCV)
      ADD_SUBDIRECTORY(forestmodevis)
    ENDIF()
  ENDIF()
ENDIF()

IF(BUILD_SPAINT)
  ADD_SUBDIRECTORY(spaintgui)
ENDIF()
