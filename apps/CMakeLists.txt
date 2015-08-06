##################################
# CMakeLists.txt for spaint/apps #
##################################

IF(BUILD_AUXILIARY_APPS AND BUILD_EVALUATION_MODULES)
  ADD_SUBDIRECTORY(raflperf)

  IF(WITH_OPENCV)
    ADD_SUBDIRECTORY(raflvis)

    IF(WITH_ARRAYFIRE)
      ADD_SUBDIRECTORY(touchtrain)
    ENDIF()
  ENDIF()
ENDIF()

ADD_SUBDIRECTORY(spaintgui)
