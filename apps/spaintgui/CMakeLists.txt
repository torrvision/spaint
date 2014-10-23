#####################################
# CMakeLists.txt for apps/spaintgui #
#####################################

###########################
# Specify the target name #
###########################

SET(targetname spaintgui)

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

IF(WITH_CUDA)
  SET(sources main.cpp Application.cpp CUDAInstantiations.cu)
ELSE()
  SET(sources main.cpp Application.cpp)
ENDIF()
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

TARGET_LINK_LIBRARIES(${targetname} spaint)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkSDL.cmake)

#############################
# Specify things to install #
#############################

INCLUDE(${PROJECT_SOURCE_DIR}/InstallApp.cmake)
