###################
# LinkGrove.cmake #
###################

IF(BUILD_GROVE)
  TARGET_LINK_LIBRARIES(${targetname} grove tvgutil cudadevrt)

  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkScoreForests.cmake)
ENDIF()
