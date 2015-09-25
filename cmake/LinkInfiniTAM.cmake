#######################
# LinkInfiniTAM.cmake #
#######################

TARGET_LINK_LIBRARIES(${targetname} Engine ITMLib Utils)

IF(PNG_FOUND)
  TARGET_LINK_LIBRARIES(${targetname} ${PNG_LIBRARIES})
ENDIF()
