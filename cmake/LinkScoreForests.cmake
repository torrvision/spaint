##########################
# LinkScoreForests.cmake #
##########################

# Scoreforests needs C++11 support.
IF(WITH_SCOREFORESTS AND WITH_C++11)
  TARGET_LINK_LIBRARIES(${targetname} ${ScoreForests_LIBRARIES})
  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkALGLIB.cmake)
ENDIF()
