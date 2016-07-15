#####################################
# CMakeLists.txt for spaint/modules #
#####################################

ADD_SUBDIRECTORY(tvginput)
ADD_SUBDIRECTORY(tvgutil)
ADD_SUBDIRECTORY(tvgplot)

IF(BUILD_EVALUATION_MODULES)
  ADD_SUBDIRECTORY(evaluation)
  ADD_SUBDIRECTORY(raflevaluation)
ENDIF()

IF(BUILD_INFERMOUS)
  ADD_SUBDIRECTORY(infermous)
ENDIF()

ADD_SUBDIRECTORY(rafl)
ADD_SUBDIRECTORY(rigging)

ADD_SUBDIRECTORY(spaint)

# Note: Ordering is important on Linux!
