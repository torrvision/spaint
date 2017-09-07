#################
# LinkVTK.cmake #
#################

IF(WITH_VTK)
  TARGET_LINK_LIBRARIES(${targetname} ${VTK_LIBRARIES})
ENDIF()
