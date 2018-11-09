##################################
# CMakeLists.txt for spaint/apps #
##################################

IF(BUILD_AUXILIARY_APPS)
  ADD_SUBDIRECTORY(combineglobalposes)

  IF(BUILD_EVALUATION_MODULES AND BUILD_SPAINT AND WITH_ARRAYFIRE AND WITH_OPENCV)
    ADD_SUBDIRECTORY(touchtrain)
  ENDIF()

  IF(BUILD_GROVE AND BUILD_GROVE_APPS)
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
      IF(WITH_OPENCV)
        ADD_SUBDIRECTORY(prepare_7scenes)
      ENDIF()

      ADD_SUBDIRECTORY(relocicpeval)
      ADD_SUBDIRECTORY(relocnovelposes)
      ADD_SUBDIRECTORY(relocopt)
      ADD_SUBDIRECTORY(relocperf)
    ENDIF()
  ENDIF()

  IF(BUILD_RAFL_APPS)
    IF(BUILD_EVALUATION_MODULES)
      ADD_SUBDIRECTORY(raflperf)

      IF(WITH_OPENCV)
        ADD_SUBDIRECTORY(raflvis)
      ENDIF()
    ENDIF()
  ENDIF()
ENDIF()

IF(BUILD_GROVE AND BUILD_GROVE_APPS AND WITH_OPENCV)
  ADD_SUBDIRECTORY(relocgui)
ENDIF()

IF(BUILD_SPAINT AND BUILD_SPAINT_APPS)
  ADD_SUBDIRECTORY(spaintgui)
ENDIF()
