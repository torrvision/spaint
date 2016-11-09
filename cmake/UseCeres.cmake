##################
# UseCeres.cmake #
##################

OPTION(WITH_CERES "Build with Ceres Solver support?" OFF)

IF(WITH_CERES)
  SET(GLOG_ROOT "${PROJECT_SOURCE_DIR}/libraries/glog-0.3.4")

  IF(MSVC_IDE)
    FIND_PATH(GLOG_INCLUDE_DIR glog/logging.h HINTS "${GLOG_ROOT}/src/windows")
    FIND_LIBRARY(GLOG_LIBRARY_DEBUG libglog.lib HINTS "${GLOG_ROOT}/Debug")
    FIND_LIBRARY(GLOG_LIBRARY_RELEASE libglog.lib HINTS "${GLOG_ROOT}/Release")
    SET(GLOG_LIBRARY debug ${GLOG_LIBRARY_DEBUG} optimized ${GLOG_LIBRARY_RELEASE})
  ELSE()
    IF(APPLE)
      FIND_LIBRARY(ACCELERATE_LIBRARY Accelerate)
    ENDIF()

    FIND_PATH(GLOG_INCLUDE_DIR glog/logging.h HINTS "${GLOG_ROOT}/installed/include")
    FIND_LIBRARY(GLOG_LIBRARY glog HINTS "${GLOG_ROOT}/installed/lib")
  ENDIF()

  INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIR})

  SET(Ceres_DIR "${PROJECT_SOURCE_DIR}/libraries/ceres-solver-1.11.0/install/share/Ceres")
  FIND_PACKAGE(Ceres REQUIRED)

  INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

  ADD_DEFINITIONS(-DWITH_CERES)
ENDIF()
