###################
# LinkTorch.cmake #
###################

IF(WITH_TORCH)
  TARGET_LINK_LIBRARIES(${targetname} ${TORCH_LIBRARIES})
ENDIF()
