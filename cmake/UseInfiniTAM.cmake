######################
# UseInfiniTAM.cmake #
######################

SET(InfiniTAM_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM" CACHE PATH "The InfiniTAM include directory")
SET(InfiniTAM_BUILD_DIR "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM/build" CACHE PATH "The InfiniTAM build directory")

FIND_LIBRARY(InfiniTAM_InputSource_LIBRARY InputSource HINTS "${InfiniTAM_BUILD_DIR}/InputSource")
FIND_LIBRARY(InfiniTAM_ITMLib_LIBRARY ITMLib HINTS "${InfiniTAM_BUILD_DIR}/ITMLib")
FIND_LIBRARY(InfiniTAM_ORUtils_LIBRARY ORUtils HINTS "${InfiniTAM_BUILD_DIR}/ORUtils")
FIND_LIBRARY(InfiniTAM_RelocLib_LIBRARY RelocLib HINTS "${InfiniTAM_BUILD_DIR}/RelocLib")
SET(InfiniTAM_LIBRARIES ${InfiniTAM_InputSource_LIBRARY} ${InfiniTAM_ITMLib_LIBRARY} ${InfiniTAM_ORUtils_LIBRARY} ${InfiniTAM_RelocLib_LIBRARY})

INCLUDE_DIRECTORIES(${InfiniTAM_INCLUDE_DIR})

ADD_DEFINITIONS(-DUSING_CMAKE)

IF(NOT(WITH_CUDA))
  ADD_DEFINITIONS(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

# Search for libpng and automatically use it if found - this mirrors what is done by InfiniTAM.
FIND_PACKAGE(PNG)

IF(PNG_FOUND)
  INCLUDE_DIRECTORIES(${PNG_INCLUDE_DIRS})
  ADD_DEFINITIONS(${PNG_DEFINITIONS})

  # Note: This preprocessor flag is used within InfiniTAM to control the conditional compilation of code using libpng.
  ADD_DEFINITIONS(-DUSE_LIBPNG)
ENDIF()
