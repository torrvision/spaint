####################################
# CMakeLists.txt for scratch/eigen #
####################################

###########################
# Specify the target name #
###########################

SET(targetname scratchtest_eigen)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)

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

# None currently needed

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetScratchTestTarget.cmake)

#################################
# Specify the libraries to link #
#################################

# None currently needed
