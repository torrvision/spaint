#######################
# LinkArrayFire.cmake #
#######################

IF(WITH_ARRAYFIRE)
  IF(WITH_CUDA)
    TARGET_LINK_LIBRARIES(${targetname} ${ArrayFire_CUDA_LIBRARIES})
    IF(MSVC_IDE)
      ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CUDA_TOOLKIT_ROOT_DIR}/nvvm/bin/nvvm64_30_0.dll" "$<TARGET_FILE_DIR:${targetname}>")
    ENDIF()
  ELSE()
    TARGET_LINK_LIBRARIES(${targetname} ${ArrayFire_CPU_LIBRARIES})
  ENDIF()
ENDIF()
