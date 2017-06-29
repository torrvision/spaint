##################################
# CMakeLists.txt for spaint/apps #
##################################

IF(BUILD_AUXILIARY_APPS)
  IF(BUILD_EVALUATION_MODULES)
    ADD_SUBDIRECTORY(raflperf)

    IF(WITH_OPENCV)
      ADD_SUBDIRECTORY(raflvis)

      IF(BUILD_SPAINT AND WITH_ARRAYFIRE)
        ADD_SUBDIRECTORY(touchtrain)
      ENDIF()
    ENDIF()
  ENDIF()

  IF(BUILD_GROVE)
    IF(WITH_SCOREFORESTS)
      ADD_SUBDIRECTORY(relocconverter)
    ENDIF()

    IF(WITH_OPENCV)
      IF(WITH_VTK)
        ADD_SUBDIRECTORY(forestmodevis)
      ENDIF()

      IF(BUILD_RELOCVIS)
        ADD_SUBDIRECTORY(relocvis)
      ENDIF()
    ENDIF()

    IF(BUILD_EVALUATION_MODULES)
      ADD_SUBDIRECTORY(prepare_7scenes)
      ADD_SUBDIRECTORY(relocperf)
    ENDIF()
  ENDIF()
ENDIF()

IF(BUILD_GROVE)
  ADD_SUBDIRECTORY(relocgui)
ENDIF()

IF(BUILD_SPAINT)
  ADD_SUBDIRECTORY(spaintgui)
ENDIF()
