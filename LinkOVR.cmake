#################
# LinkOVR.cmake #
#################

IF(WITH_OVR)
  TARGET_LINK_LIBRARIES(${targetname} ${OVR_LIBRARY})

  IF(APPLE)
    TARGET_LINK_LIBRARIES(${targetname} ${COCOA} ${COREFOUNDATION} ${COREGRAPHICS} ${IOKIT})
  ENDIF()

  IF(MSVC_IDE)
    TARGET_LINK_LIBRARIES(${targetname} winmm ws2_32)
  ENDIF()
ENDIF()
