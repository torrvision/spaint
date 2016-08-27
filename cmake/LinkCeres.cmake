###################
# LinkCeres.cmake #
###################

IF(WITH_CERES)
  IF(APPLE)
    TARGET_LINK_LIBRARIES(${targetname} ${ACCELERATE_LIBRARY})
  ENDIF()

  TARGET_LINK_LIBRARIES(${targetname} ${CERES_LIBRARY} ${GLOG_LIBRARY})

  IF(MSVC_IDE)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${GLOG_ROOT}/x64/Release/libglog.dll" "$<TARGET_FILE_DIR:${targetname}>")
  ENDIF()
ENDIF()
