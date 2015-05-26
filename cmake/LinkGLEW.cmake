##################
# LinkGLEW.cmake #
##################

IF(MSVC_IDE)
  TARGET_LINK_LIBRARIES(${targetname} ${GLEW_LIBRARY})
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${GLEW_ROOT}/bin/Release/x64/glew32.dll" "$<TARGET_FILE_DIR:${targetname}>")
ENDIF()
