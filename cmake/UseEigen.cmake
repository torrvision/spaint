##################
# UseEigen.cmake #
##################

FIND_PATH(EIGEN_INCLUDE_DIR eigen3.pc.in HINTS "${PROJECT_SOURCE_DIR}/libraries/Eigen-3.2.2")
INCLUDE_DIRECTORIES(SYSTEM ${EIGEN_INCLUDE_DIR})
