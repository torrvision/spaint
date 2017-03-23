##################
# UseGrove.cmake #
##################

IF(BUILD_GROVE)
  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseScoreForests.cmake)

  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/grove/include)
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

  ADD_DEFINITIONS(-DWITH_GROVE)
ENDIF()
