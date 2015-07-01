#################
# LinkTBB.cmake #
#################

IF(WITH_TBB)
  TARGET_LINK_LIBRARIES(${targetname} ${TBB_LIBRARY})

  IF(MSVC_IDE)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${TBB_ROOT}/bin/intel64/vc12/tbb.dll "$<TARGET_FILE_DIR:${targetname}>")
  ENDIF()
ENDIF()
