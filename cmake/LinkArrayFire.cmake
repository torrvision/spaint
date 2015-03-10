#######################
# LinkArrayFire.cmake #
#######################

IF(WITH_ARRAYFIRE)
  TARGET_LINK_LIBRARIES(${targetname} ${ArrayFire_LIBRARIES})
ENDIF()
