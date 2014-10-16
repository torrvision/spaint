###########################################
# CMakeLists.txt for tests/test-infinitam #
###########################################

###########################
# Specify the target name #
###########################

SET(targetname test-infinitam)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseInfiniTAM.cmake)

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

# None needed

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/SetCUDATestTarget.cmake)

#################################
# Specify the libraries to link #
#################################

INCLUDE(${PROJECT_SOURCE_DIR}/LinkInfiniTAM.cmake)
