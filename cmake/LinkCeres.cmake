###################
# LinkCeres.cmake #
###################

IF(WITH_CERES)
  TARGET_LINK_LIBRARIES(${targetname} ${ACCELERATE_LIBRARY} ${CERES_LIBRARY} ${GLOG_LIBRARY})
ENDIF()
