##################
# UseCeres.cmake #
##################

OPTION(WITH_CERES "Build with Ceres Solver support?" OFF)

IF(WITH_CERES)
  IF(MSVC_IDE)
    FIND_PATH(GLOG_INCLUDE_DIR glog/logging.h HINTS "${PROJECT_SOURCE_DIR}/libraries/glog-0.3.4/src/windows")
    FIND_LIBRARY(GLOG_LIBRARY libglog.lib HINTS "${PROJECT_SOURCE_DIR}/libraries/glog-0.3.4/x64/Release")
  ELSEIF(APPLE)
    FIND_LIBRARY(ACCELERATE_LIBRARY Accelerate)

    FIND_PATH(GLOG_INCLUDE_DIR glog/logging.h HINTS "${PROJECT_SOURCE_DIR}/libraries/glog-0.3.4/installed/include")
    FIND_LIBRARY(GLOG_LIBRARY glog HINTS "${PROJECT_SOURCE_DIR}/libraries/glog-0.3.4/installed/lib")
  ENDIF()

  INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIR})

  IF(MSVC_IDE)
    FIND_PATH(CERES_CONFIG_DIR ceres/internal/config.h HINTS "${PROJECT_SOURCE_DIR}/libraries/ceres-solver-1.11.0/build/config")
    FIND_PATH(CERES_INCLUDE_DIR ceres/ceres.h HINTS "${PROJECT_SOURCE_DIR}/libraries/ceres-solver-1.11.0/include")
    FIND_LIBRARY(CERES_LIBRARY ceres HINTS "${PROJECT_SOURCE_DIR}/libraries/ceres-solver-1.11.0/build/x64/Release")
  ELSE()
    FIND_PATH(CERES_INCLUDE_DIR ceres/ceres.h HINTS "${PROJECT_SOURCE_DIR}/libraries/ceres-solver-1.11.0/install/include")
    FIND_LIBRARY(CERES_LIBRARY ceres HINTS "${PROJECT_SOURCE_DIR}/libraries/ceres-solver-1.11.0/install/lib")
  ENDIF()

  INCLUDE_DIRECTORIES(${CERES_CONFIG_DIR} ${CERES_INCLUDE_DIR})

  ADD_DEFINITIONS(-DWITH_CERES)
ENDIF()
