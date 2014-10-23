#####################################
# CMakeLists.txt for tests/test-ovr #
#####################################

###########################
# Specify the target name #
###########################

SET(targetname test-ovr)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/UseOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseOVR.cmake)

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

INCLUDE(${PROJECT_SOURCE_DIR}/LinkOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkOVR.cmake)
