#################
# LinkSDL.cmake #
#################

TARGET_LINK_LIBRARIES(${targetname} ${SDL_LIBRARY} ${SDLMAIN_LIBRARY})

IF(MSVC_IDE)
  TARGET_LINK_LIBRARIES(${targetname} imm32 version winmm)
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PROJECT_SOURCE_DIR}/libraries/SDL2-2.0.7/install/bin/SDL2.dll" "$<TARGET_FILE_DIR:${targetname}>")
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PROJECT_SOURCE_DIR}/libraries/SDL2-2.0.7/install/bin/SDL2d.dll" "$<TARGET_FILE_DIR:${targetname}>")
ENDIF()
