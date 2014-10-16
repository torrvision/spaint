###################
# UseOpenCV.cmake #
###################

FIND_PACKAGE(OpenCV 2.4 REQUIRED HINTS "${PROJECT_SOURCE_DIR}/libraries/opencv-2.4.9/build")
INCLUDE_DIRECTORIES(SYSTEM ${OpenCV_INCLUDE_DIR})
