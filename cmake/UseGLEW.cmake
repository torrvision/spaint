#################
# UseGLEW.cmake #
#################

IF(MSVC_IDE)
  FIND_PATH(GLEW_INCLUDE_DIR "GL/glew.h" HINTS "${PROJECT_SOURCE_DIR}/libraries/glew-1.12.0/include")
  FIND_LIBRARY(GLEW_LIBRARY glew32 HINTS "${PROJECT_SOURCE_DIR}/libraries/glew-1.12.0/lib/Release/x64")
  INCLUDE_DIRECTORIES(${GLEW_INCLUDE_DIR})
ENDIF()
