#################################
# CMakeLists.txt for tests/unit #
#################################

IF(BUILD_EVALUATION)
  ADD_SUBDIRECTORY(evaluation)
ENDIF()

ADD_SUBDIRECTORY(infermous)
ADD_SUBDIRECTORY(rafl)
ADD_SUBDIRECTORY(rigging)
ADD_SUBDIRECTORY(tvgutil)
