#######################
# LinkRealSense.cmake #
#######################

IF(WITH_REALSENSE)
  TARGET_LINK_LIBRARIES(${targetname} ${RealSense_LIBRARY})
  IF(MSVC_IDE)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${RealSense_ROOT}/bin/x64/realsense.dll" "$<TARGET_FILE_DIR:${targetname}>")
  ENDIF()
ENDIF()
