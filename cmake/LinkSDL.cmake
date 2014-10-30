#################
# LinkSDL.cmake #
#################

TARGET_LINK_LIBRARIES(${targetname} ${SDL_LIBRARY} ${SDLMAIN_LIBRARY})

IF(MSVC_IDE)
  TARGET_LINK_LIBRARIES(${targetname} imm32 version winmm)
ENDIF()
