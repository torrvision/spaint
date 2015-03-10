########################################
# CMakeLists.txt for scratch/arrayfire #
########################################

###########################
# Specify the target name #
###########################

SET(targetname scratchtest_arrayfire)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseArrayFire.cmake)

#############################
# Specify the project files #
#############################

SET(sources main.cpp)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(sources FILES ${sources})

##########################################
# Specify additional include directories #
##########################################

#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetScratchTestTarget.cmake)

#################################
# Specify the libraries to link #
#################################

#TARGET_LINK_LIBRARIES(${targetname})
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkArrayFire.cmake)
