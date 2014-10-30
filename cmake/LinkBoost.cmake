###################
# LinkBoost.cmake #
###################

IF(WITH_BOOST)
  TARGET_LINK_LIBRARIES(${targetname} ${Boost_LIBRARIES})
ENDIF()
