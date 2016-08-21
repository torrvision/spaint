##################
# UseCeres.cmake #
##################

OPTION(WITH_CERES "Build with Ceres Solver support?" OFF)

IF(WITH_CERES)
  # Mac OS X-specific
  FIND_LIBRARY(ACCELERATE_LIBRARY Accelerate)

  FIND_PATH(GLOG_INCLUDE_DIR glog/logging.h HINTS "${PROJECT_SOURCE_DIR}/libraries/glog-0.3.4/installed/include")
  FIND_LIBRARY(GLOG_LIBRARY glog HINTS "${PROJECT_SOURCE_DIR}/libraries/glog-0.3.4/installed/lib")
  INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIR})

  FIND_PATH(CERES_INCLUDE_DIR ceres/ceres.h HINTS "${PROJECT_SOURCE_DIR}/libraries/ceres-solver-1.11.0/install/include")
  FIND_LIBRARY(CERES_LIBRARY ceres HINTS "${PROJECT_SOURCE_DIR}/libraries/ceres-solver-1.11.0/install/lib")
  INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIR})

  ADD_DEFINITIONS(-DWITH_CERES)
ENDIF()
