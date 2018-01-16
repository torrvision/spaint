#################
# LinkZed.cmake #
#################

IF(WITH_ZED)
  TARGET_LINK_LIBRARIES(${targetname} ${ZED_LIBRARIES})
ENDIF()
