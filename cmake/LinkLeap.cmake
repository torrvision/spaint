##################
# LinkLeap.cmake #
##################

IF(WITH_LEAP)
  TARGET_LINK_LIBRARIES(${targetname} ${LEAP_LIBRARY})
  IF(MSVC_IDE)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${LEAP_ROOT}/lib/x64/Leap.dll" "$<TARGET_FILE_DIR:${targetname}>")
  ELSEIF(APPLE)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${LEAP_ROOT}/lib/libLeap.dylib" "$<TARGET_FILE_DIR:${targetname}>")
  ELSE()
    TARGET_LINK_LIBRARIES(${targetname} z)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${LEAP_ROOT}/lib/x64/libLeap.so" "$<TARGET_FILE_DIR:${targetname}>")
  ENDIF()
ENDIF()
