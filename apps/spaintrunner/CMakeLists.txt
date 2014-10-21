########################################
# CMakeLists.txt for apps/spaintrunner #
########################################

###########################
# Specify the target name #
###########################

SET(targetname spaintrunner)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseSDL.cmake)

#############################
# Specify the project files #
#############################

SET(sources main.cpp Application.cpp)
SET(headers Application.h)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(sources FILES ${sources})
SOURCE_GROUP(headers FILES ${headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/SetCUDAAppTarget.cmake)

#################################
# Specify the libraries to link #
#################################

INCLUDE(${PROJECT_SOURCE_DIR}/LinkInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkSDL.cmake)
TARGET_LINK_LIBRARIES(${targetname} spaint)

#############################
# Specify things to install #
#############################

INCLUDE(${PROJECT_SOURCE_DIR}/InstallApp.cmake)
