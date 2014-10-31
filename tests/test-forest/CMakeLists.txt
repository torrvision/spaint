#######################################
# CMakeLists.txt for tests/test-forest #
#######################################

###########################
# Specify the target name #
###########################

SET(targetname test-forest)

################################
# Specify the libraries to use #
################################

#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)

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

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/forest/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetTestTarget.cmake)

#################################
# Specify the libraries to link #
#################################
TARGET_LINK_LIBRARIES(${targetname} forest)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)
