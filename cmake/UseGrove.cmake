##################
# UseGrove.cmake #
##################

IF(BUILD_GROVE)
  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseALGLIB.cmake)
  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseScoreForests.cmake)

  IF(NOT WITH_ALGLIB)
    MESSAGE(FATAL_ERROR "Grove needs WITH_ALGLIB be enabled.")
  ENDIF()

  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/grove/include)
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

  ADD_DEFINITIONS(-DWITH_GROVE)
ENDIF()
