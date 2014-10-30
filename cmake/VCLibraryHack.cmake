#######################
# VCLibraryHack.cmake #
#######################

IF(MSVC_IDE)
  SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:\"libcmtd.lib;msvcrt.lib\"")
  SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:\"libcmt.lib\"")
ENDIF(MSVC_IDE)

