###################
# LinkCeres.cmake #
###################

IF(WITH_CERES)
  IF(APPLE)
    TARGET_LINK_LIBRARIES(${targetname} ${ACCELERATE_LIBRARY})
  ENDIF()

  TARGET_LINK_LIBRARIES(${targetname} ${CERES_LIBRARY} ${GLOG_LIBRARY})
ENDIF()
