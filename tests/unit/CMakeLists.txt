#################################
# CMakeLists.txt for tests/unit #
#################################

IF(BUILD_EVALUATION_MODULES)
  ADD_SUBDIRECTORY(evaluation)
ENDIF()

IF(BUILD_INFERMOUS)
  ADD_SUBDIRECTORY(infermous)
ENDIF()

ADD_SUBDIRECTORY(rafl)
ADD_SUBDIRECTORY(rigging)
ADD_SUBDIRECTORY(tvgutil)
