####################
# LinkOpenNI.cmake #
####################

IF(WITH_OPENNI)
  TARGET_LINK_LIBRARIES(${targetname} ${OpenNI_LIBRARIES})
  IF(MSVC_IDE)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPEN_NI_ROOT}/Redist/OpenNI2.dll" $<TARGET_FILE_DIR:${targetname}>)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${OPEN_NI_ROOT}/Redist/OpenNI2" "$<TARGET_FILE_DIR:${targetname}>/OpenNI2")
  ENDIF()
ENDIF()
