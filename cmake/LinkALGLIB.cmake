####################
# LinkALGLIB.cmake #
####################

IF(WITH_ALGLIB)
  TARGET_LINK_LIBRARIES(${targetname} ${ALGLIB_LIBRARY})
ENDIF()
