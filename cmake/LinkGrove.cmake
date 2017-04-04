###################
# LinkGrove.cmake #
###################

IF(BUILD_GROVE)
  TARGET_LINK_LIBRARIES(${targetname} grove tvgutil)

  IF(NOT MSVC_IDE)
    TARGET_LINK_LIBRARIES(${targetname} cudadevrt)
  ENDIF()

  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkScoreForests.cmake)
ENDIF()
