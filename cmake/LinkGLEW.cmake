##################
# LinkGLEW.cmake #
##################

IF(MSVC_IDE OR "${CMAKE_SYSTEM}" MATCHES "Linux")
  TARGET_LINK_LIBRARIES(${targetname} ${GLEW_LIBRARY})
ENDIF()

IF(MSVC_IDE)
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${GLEW_ROOT}/bin/Release/x64/glew32.dll" "$<TARGET_FILE_DIR:${targetname}>")
ENDIF()
