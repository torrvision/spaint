#################
# UseGLEW.cmake #
#################

IF(MSVC_IDE)
  FIND_PATH(GLEW_ROOT glew.pc.in HINTS "${PROJECT_SOURCE_DIR}/libraries/glew-1.12.0")
  FIND_PATH(GLEW_INCLUDE_DIR "GL/glew.h" HINTS "${GLEW_ROOT}/include")
  FIND_LIBRARY(GLEW_LIBRARY glew32 HINTS "${GLEW_ROOT}/lib/Release/x64")
  INCLUDE_DIRECTORIES(${GLEW_INCLUDE_DIR})
ENDIF()
