########################################
# CMakeLists.txt for tests/test-opencv #
########################################

###########################
# Specify the target name #
###########################

SET(targetname test-opencv)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/UseOpenCV.cmake)

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

INCLUDE(${PROJECT_SOURCE_DIR}/SetTestTarget.cmake)

#################################
# Specify the libraries to link #
#################################

INCLUDE(${PROJECT_SOURCE_DIR}/LinkOpenCV.cmake)
