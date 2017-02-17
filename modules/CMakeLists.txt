#####################################
# CMakeLists.txt for spaint/modules #
#####################################

ADD_SUBDIRECTORY(tvginput)

IF(WITH_OPENCV)
  ADD_SUBDIRECTORY(tvgplot)
ENDIF()

ADD_SUBDIRECTORY(tvgutil)

IF(BUILD_EVALUATION_MODULES)
  ADD_SUBDIRECTORY(evaluation)
  ADD_SUBDIRECTORY(raflevaluation)
ENDIF()

IF(BUILD_GROVE)
  ADD_SUBDIRECTORY(grove)
ENDIF()

IF(BUILD_INFERMOUS)
  ADD_SUBDIRECTORY(infermous)
ENDIF()

ADD_SUBDIRECTORY(rafl)
ADD_SUBDIRECTORY(rigging)

IF(BUILD_SPAINT)
  ADD_SUBDIRECTORY(spaint)
ENDIF()

# Note: Ordering is important on Linux!
