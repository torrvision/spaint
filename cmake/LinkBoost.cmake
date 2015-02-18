###################
# LinkBoost.cmake #
###################

TARGET_LINK_LIBRARIES(${targetname} ${Boost_LIBRARIES})

IF("${CMAKE_SYSTEM}" MATCHES "Linux")
  TARGET_LINK_LIBRARIES(${targetname} rt)
ENDIF()
