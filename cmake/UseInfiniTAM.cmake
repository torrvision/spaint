######################
# UseInfiniTAM.cmake #
######################

SET(InfiniTAM_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/../infinitam" CACHE PATH "The InfiniTAM include directory")
SET(InfiniTAM_BUILD_DIR "${PROJECT_SOURCE_DIR}/../infinitam/build" CACHE PATH "The InfiniTAM build directory")

FIND_LIBRARY(InfiniTAM_FernRelocLib_LIBRARY_DEBUG FernRelocLib HINTS "${InfiniTAM_BUILD_DIR}/FernRelocLib" "${InfiniTAM_BUILD_DIR}/FernRelocLib/Debug")
FIND_LIBRARY(InfiniTAM_InputSource_LIBRARY_DEBUG InputSource HINTS "${InfiniTAM_BUILD_DIR}/InputSource" "${InfiniTAM_BUILD_DIR}/InputSource/Debug")
FIND_LIBRARY(InfiniTAM_ITMLib_LIBRARY_DEBUG ITMLib HINTS "${InfiniTAM_BUILD_DIR}/ITMLib" "${InfiniTAM_BUILD_DIR}/ITMLib/Debug")
FIND_LIBRARY(InfiniTAM_MiniSlamGraphLib_LIBRARY_DEBUG MiniSlamGraphLib HINTS "${InfiniTAM_BUILD_DIR}/MiniSlamGraphLib" "${InfiniTAM_BUILD_DIR}/MiniSlamGraphLib/Debug")
FIND_LIBRARY(InfiniTAM_ORUtils_LIBRARY_DEBUG ORUtils HINTS "${InfiniTAM_BUILD_DIR}/ORUtils" "${InfiniTAM_BUILD_DIR}/ORUtils/Debug")

FIND_LIBRARY(InfiniTAM_FernRelocLib_LIBRARY_RELEASE FernRelocLib HINTS "${InfiniTAM_BUILD_DIR}/FernRelocLib" "${InfiniTAM_BUILD_DIR}/FernRelocLib/Release")
FIND_LIBRARY(InfiniTAM_InputSource_LIBRARY_RELEASE InputSource HINTS "${InfiniTAM_BUILD_DIR}/InputSource" "${InfiniTAM_BUILD_DIR}/InputSource/Release")
FIND_LIBRARY(InfiniTAM_ITMLib_LIBRARY_RELEASE ITMLib HINTS "${InfiniTAM_BUILD_DIR}/ITMLib" "${InfiniTAM_BUILD_DIR}/ITMLib/Release")
FIND_LIBRARY(InfiniTAM_MiniSlamGraphLib_LIBRARY_RELEASE MiniSlamGraphLib HINTS "${InfiniTAM_BUILD_DIR}/MiniSlamGraphLib" "${InfiniTAM_BUILD_DIR}/MiniSlamGraphLib/Release")
FIND_LIBRARY(InfiniTAM_ORUtils_LIBRARY_RELEASE ORUtils HINTS "${InfiniTAM_BUILD_DIR}/ORUtils" "${InfiniTAM_BUILD_DIR}/ORUtils/Release")

SET(InfiniTAM_FernRelocLib_LIBRARY debug ${InfiniTAM_FernRelocLib_LIBRARY_DEBUG} optimized ${InfiniTAM_FernRelocLib_LIBRARY_RELEASE})
SET(InfiniTAM_InputSource_LIBRARY debug ${InfiniTAM_InputSource_LIBRARY_DEBUG} optimized ${InfiniTAM_InputSource_LIBRARY_RELEASE})
SET(InfiniTAM_ITMLib_LIBRARY debug ${InfiniTAM_ITMLib_LIBRARY_DEBUG} optimized ${InfiniTAM_ITMLib_LIBRARY_RELEASE})
SET(InfiniTAM_MiniSlamGraphLib_LIBRARY debug ${InfiniTAM_MiniSlamGraphLib_LIBRARY_DEBUG} optimized ${InfiniTAM_MiniSlamGraphLib_LIBRARY_RELEASE})
SET(InfiniTAM_ORUtils_LIBRARY debug ${InfiniTAM_ORUtils_LIBRARY_DEBUG} optimized ${InfiniTAM_ORUtils_LIBRARY_RELEASE})

SET(InfiniTAM_LIBRARIES ${InfiniTAM_FernRelocLib_LIBRARY} ${InfiniTAM_InputSource_LIBRARY} ${InfiniTAM_ITMLib_LIBRARY} ${InfiniTAM_MiniSlamGraphLib_LIBRARY} ${InfiniTAM_ORUtils_LIBRARY})

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
