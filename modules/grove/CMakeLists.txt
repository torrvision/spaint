####################################
# CMakeLists.txt for modules/grove #
####################################

###########################
# Specify the target name #
###########################

SET(targetname grove)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)

#############################
# Specify the project files #
#############################

##
SET(features_base_headers include/grove/features/base/Descriptor.h)

##
SET(keypoints_headers
include/grove/keypoints/Keypoint2D.h
include/grove/keypoints/Keypoint3DColour.h
)

##
SET(numbers_sources
src/numbers/CPURNG.cpp
)

SET(numbers_headers
include/grove/numbers/CPURNG.h
include/grove/numbers/CUDARNG.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${numbers_sources}
)

SET(headers
${features_base_headers}
${keypoints_headers}
${numbers_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(features\\base FILES ${features_base_headers})
SOURCE_GROUP(keypoints FILES ${keypoints_headers})
SOURCE_GROUP(numbers FILES ${numbers_sources} ${numbers_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/grove)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/helpers)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/itmx/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
