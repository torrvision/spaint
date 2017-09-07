###################
# LinkGrove.cmake #
###################

IF(BUILD_GROVE)
  TARGET_LINK_LIBRARIES(${targetname} grove itmx tvgutil ${CUDA_cudadevrt_LIBRARY})

  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkALGLIB.cmake)
  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkScoreForests.cmake)
ENDIF()
