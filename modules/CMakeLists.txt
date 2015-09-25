#####################################
# CMakeLists.txt for spaint/modules #
#####################################

ADD_SUBDIRECTORY(tvginput)
ADD_SUBDIRECTORY(tvgutil)

IF(BUILD_EVALUATION_MODULES)
  ADD_SUBDIRECTORY(evaluation)
  ADD_SUBDIRECTORY(raflevaluation)
ENDIF()

ADD_SUBDIRECTORY(infermous)
ADD_SUBDIRECTORY(rafl)
ADD_SUBDIRECTORY(rigging)

ADD_SUBDIRECTORY(spaint)

# Note: Ordering is important on Linux!
