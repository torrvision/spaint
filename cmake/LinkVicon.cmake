###################
# LinkVicon.cmake #
###################

IF(WITH_VICON)
  TARGET_LINK_LIBRARIES(${targetname} ${VICON_DATASTREAM_LIBRARY})

  IF(MSVC_IDE)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${VICON_ROOT}/lib/boost_chrono-vc110-mt-1_55.dll "$<TARGET_FILE_DIR:${targetname}>")
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${VICON_ROOT}/lib/boost_system-vc110-mt-1_55.dll "$<TARGET_FILE_DIR:${targetname}>")
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${VICON_ROOT}/lib/boost_thread-vc110-mt-1_55.dll "$<TARGET_FILE_DIR:${targetname}>")
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${VICON_ROOT}/lib/ViconDataStreamSDK_CPP.dll "$<TARGET_FILE_DIR:${targetname}>")
  ELSE()
    TARGET_LINK_LIBRARIES(${targetname} ${VICON_DEBUGSERVICES_LIBRARY})
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${VICON_DATASTREAM_LIBRARY} $<TARGET_FILE_DIR:${targetname}>)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${VICON_DEBUGSERVICES_LIBRARY} $<TARGET_FILE_DIR:${targetname}>)
  ENDIF()
ENDIF()
