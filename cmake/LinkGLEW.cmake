##################
# LinkGLEW.cmake #
##################

IF(MSVC_IDE)
  TARGET_LINK_LIBRARIES(${targetname} ${GLEW_LIBRARY})
ENDIF()
