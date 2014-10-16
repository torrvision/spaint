######################
# UseInfiniTAM.cmake #
######################

SET(InfiniTAM_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM" CACHE FILEPATH "The InfiniTAM include directory")
SET(InfiniTAM_Engine_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM/build/Engine" CACHE FILEPATH "The directory containing the Engine library")
SET(InfiniTAM_ITMLib_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM/build/ITMLib" CACHE FILEPATH "The directory containing the ITMLib library")
SET(InfiniTAM_Utils_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/../InfiniTAM/InfiniTAM/build/Utils" CACHE FILEPATH "The directory containing the Utils library")
INCLUDE_DIRECTORIES(${InfiniTAM_INCLUDE_DIR})
LINK_DIRECTORIES(${InfiniTAM_Engine_LIBRARY_DIR} ${InfiniTAM_ITMLib_LIBRARY_DIR} ${InfiniTAM_Utils_LIBRARY_DIR})
ADD_DEFINITIONS(-DUSING_CMAKE)
