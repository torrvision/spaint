##################
# LinkCUDA.cmake #
##################

IF(WITH_CUDA AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${targetname} ${CUDA_CUDA_LIBRARY})
ENDIF()
