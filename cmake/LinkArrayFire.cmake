#######################
# LinkArrayFire.cmake #
#######################

IF(WITH_ARRAYFIRE)
  IF(WITH_CUDA)
    TARGET_LINK_LIBRARIES(${targetname} ${ArrayFire_CPU_LIBRARIES} ${ArrayFire_CUDA_LIBRARIES})
  ELSE()
    TARGET_LINK_LIBRARIES(${targetname} ${ArrayFire_CPU_LIBRARIES})
  ENDIF()
ENDIF()
