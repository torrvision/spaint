#######################
# LinkInfiniTAM.cmake #
#######################

TARGET_LINK_LIBRARIES(${targetname} InputSource ITMLib ORUtils RelocLib)

IF(PNG_FOUND)
  TARGET_LINK_LIBRARIES(${targetname} ${PNG_LIBRARIES})
ENDIF()
