###################
# UseOpenCV.cmake #
###################

OPTION(WITH_OPENCV "Build with OpenCV support?" OFF)

IF(WITH_OPENCV)
  FIND_PACKAGE(OpenCV 2.4 REQUIRED HINTS "${PROJECT_SOURCE_DIR}/libraries/opencv-2.4.9/build")
  INCLUDE_DIRECTORIES(SYSTEM ${OpenCV_INCLUDE_DIR})
  ADD_DEFINITIONS(-DWITH_OPENCV)
ENDIF()
