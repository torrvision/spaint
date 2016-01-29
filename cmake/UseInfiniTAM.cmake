######################
# UseInfiniTAM.cmake #
######################

SET(InfiniTAM_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM" CACHE FILEPATH "The InfiniTAM include directory")
SET(InfiniTAM_Engine_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM/build/Engine" CACHE FILEPATH "The directory containing the Engine library")
SET(InfiniTAM_ITMLib_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM/build/ITMLib" CACHE FILEPATH "The directory containing the ITMLib library")
SET(InfiniTAM_LCDLib_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM/build/LCDLib" CACHE FILEPATH "The directory containing the LCDLib library")
SET(InfiniTAM_ORUtils_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM/build/ORUtils" CACHE FILEPATH "The directory containing the ORUtils library")
SET(InfiniTAM_Utils_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM/build/Utils" CACHE FILEPATH "The directory containing the Utils library")
INCLUDE_DIRECTORIES(${InfiniTAM_INCLUDE_DIR})
LINK_DIRECTORIES(${InfiniTAM_Engine_LIBRARY_DIR} ${InfiniTAM_ITMLib_LIBRARY_DIR} ${InfiniTAM_LCDLib_LIBRARY_DIR} ${InfiniTAM_ORUtils_LIBRARY_DIR} ${InfiniTAM_Utils_LIBRARY_DIR})
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
