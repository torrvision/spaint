##################################
# CMakeLists.txt for spaint/apps #
##################################

IF(BUILD_AUXILIARY_APPS)
  IF(BUILD_EVALUATION_MODULES)
    ADD_SUBDIRECTORY(raflperf)

    IF(WITH_OPENCV)
      ADD_SUBDIRECTORY(raflvis)

      IF(WITH_ARRAYFIRE)
        ADD_SUBDIRECTORY(touchtrain)
      ENDIF()
    ENDIF()
  ENDIF()

  IF(WITH_CERES AND WITH_OPENCV)
    ADD_SUBDIRECTORY(fitvis)
  ENDIF()
ENDIF()

ADD_SUBDIRECTORY(spaintgui)
