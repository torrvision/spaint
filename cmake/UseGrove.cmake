##################
# UseGrove.cmake #
##################

IF(BUILD_GROVE)
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/grove/include)
  ADD_DEFINITIONS(-DWITH_GROVE)
ENDIF()
