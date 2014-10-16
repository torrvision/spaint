#####################################
# CMakeLists.txt for tests/test-sdl #
#####################################

###########################
# Specify the target name #
###########################

SET(targetname test-sdl)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/UseSDL.cmake)

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

INCLUDE(${PROJECT_SOURCE_DIR}/LinkSDL.cmake)
