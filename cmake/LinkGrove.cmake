###################
# LinkGrove.cmake #
###################

IF(BUILD_GROVE)
  TARGET_LINK_LIBRARIES(${targetname} grove itmx orx tvgutil ${CUDA_cudadevrt_LIBRARY})

  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkALGLIB.cmake)
ENDIF()
