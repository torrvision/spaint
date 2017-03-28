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

  IF(MSVC_IDE)
    FIND_PATH(CERES_ROOT LICENSE HINTS "${PROJECT_SOURCE_DIR}/libraries/ceres-solver-1.11.0")
    FIND_PATH(CERES_INCLUDE_DIRS ceres/ceres.h HINTS "${CERES_ROOT}/install/include")
    FIND_LIBRARY(CERES_LIBRARY_DEBUG ceres-debug HINTS "${CERES_ROOT}/install/lib")
    FIND_LIBRARY(CERES_LIBRARY_RELEASE ceres HINTS "${CERES_ROOT}/install/lib")
    SET(CERES_LIBRARIES debug ${CERES_LIBRARY_DEBUG} optimized ${CERES_LIBRARY_RELEASE})
  ELSE()
    SET(Ceres_DIR "${PROJECT_SOURCE_DIR}/libraries/ceres-solver-1.11.0/install/share/Ceres")
    FIND_PACKAGE(Ceres REQUIRED)
  ENDIF()

  INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

  ADD_DEFINITIONS(-DWITH_CERES)
ENDIF()
