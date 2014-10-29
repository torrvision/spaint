#################
# LinkOVR.cmake #
#################

IF(WITH_OVR)
  TARGET_LINK_LIBRARIES(${targetname} ${OVR_LIBRARY})

  IF(APPLE)
    TARGET_LINK_LIBRARIES(${targetname} ${COCOA} ${COREFOUNDATION} ${COREGRAPHICS} ${IOKIT})
  ENDIF()

  IF("${CMAKE_SYSTEM}" MATCHES "Linux")
    TARGET_LINK_LIBRARIES(${targetname} pthread rt ${X11_X11_LIB} ${X11_Xrandr_LIB} ${X11_Xxf86vm_LIB})
  ENDIF()

  IF(MSVC_IDE)
    TARGET_LINK_LIBRARIES(${targetname} winmm ws2_32)
  ENDIF()
ENDIF()
