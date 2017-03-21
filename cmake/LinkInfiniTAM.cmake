#######################
# LinkInfiniTAM.cmake #
#######################

IF(WITH_INFINITAM)
  TARGET_LINK_LIBRARIES(${targetname} ${InfiniTAM_LIBRARIES})

  IF(PNG_FOUND)
    TARGET_LINK_LIBRARIES(${targetname} ${PNG_LIBRARIES})
  ENDIF()
ENDIF()
