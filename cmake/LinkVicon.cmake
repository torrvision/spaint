###################
# LinkVicon.cmake #
###################

IF(WITH_VICON)
  TARGET_LINK_LIBRARIES(${targetname} ${VICON_DATASTREAM_LIBRARY} ${VICON_DEBUGSERVICES_LIBRARY})
ENDIF()
