#################################
# CMakeLists.txt for tests/unit #
#################################

IF(BUILD_EVALUATION_MODULES)
  ADD_SUBDIRECTORY(evaluation)
ENDIF()

IF(BUILD_INFERMOUS)
  ADD_SUBDIRECTORY(infermous)
ENDIF()

ADD_SUBDIRECTORY(itmx)
ADD_SUBDIRECTORY(orx)
ADD_SUBDIRECTORY(rafl)
ADD_SUBDIRECTORY(rigging)

IF(BUILD_SPAINT)
  ADD_SUBDIRECTORY(spaint)
ENDIF()

ADD_SUBDIRECTORY(tvgutil)
