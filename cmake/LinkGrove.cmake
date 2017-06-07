###################
# LinkGrove.cmake #
###################

IF(BUILD_GROVE)
  TARGET_LINK_LIBRARIES(${targetname} grove itmx tvgutil ${CUDA_cudadevrt_LIBRARY})
ENDIF()
