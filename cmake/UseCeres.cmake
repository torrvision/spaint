##################
# UseCeres.cmake #
##################

OPTION(WITH_CERES "Build with Ceres Solver support?" OFF)

IF(WITH_CERES)
  FIND_PATH(GLOG_ROOT NEWS HINTS "${PROJECT_SOURCE_DIR}/libraries/glog-0.3.4")

  IF(MSVC_IDE)
    FIND_PATH(GLOG_INCLUDE_DIR glog/logging.h HINTS "${GLOG_ROOT}/src/windows")
    FIND_LIBRARY(GLOG_LIBRARY libglog.lib HINTS "${GLOG_ROOT}/x64/Release")
  ELSEIF(APPLE)
    FIND_LIBRARY(ACCELERATE_LIBRARY Accelerate)

    FIND_PATH(GLOG_INCLUDE_DIR glog/logging.h HINTS "${GLOG_ROOT}/installed/include")
    FIND_LIBRARY(GLOG_LIBRARY glog HINTS "${GLOG_ROOT}/installed/lib")
  ENDIF()

  INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIR})

  FIND_PATH(CERES_ROOT LICENSE HINTS "${PROJECT_SOURCE_DIR}/libraries/ceres-solver-1.11.0")
  FIND_PATH(CERES_INCLUDE_DIR ceres/ceres.h HINTS "${CERES_ROOT}/install/include")
  FIND_LIBRARY(CERES_LIBRARY ceres HINTS "${CERES_ROOT}/install/lib")

  INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIR})

  ADD_DEFINITIONS(-DWITH_CERES)
ENDIF()
