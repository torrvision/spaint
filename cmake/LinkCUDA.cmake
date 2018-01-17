##################
# LinkCUDA.cmake #
##################

IF(WITH_CUDA)
  TARGET_LINK_LIBRARIES(${targetname} ${CUDA_CUDA_LIBRARY})
ENDIF()
