##################
# UseEigen.cmake #
##################

SET(EIGEN3_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/libraries/Eigen-3.2.2" )
IF(NOT EIGEN3_INCLUDE_DIR)
MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES("${EIGEN3_INCLUDE_DIR}")
