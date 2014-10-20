####################
# LinkOpenCV.cmake #
####################

IF(WITH_OPENCV)
  TARGET_LINK_LIBRARIES(${targetname} ${OpenCV_LIBS})
ENDIF()
