####################################
# CMakeLists.txt for apps/raflperf #
####################################

###########################
# Specify the target name #
###########################

SET(targetname raflperf)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)

#############################
# Specify the project files #
#############################

##
SET(sources
main.cpp
)

SET(headers
RandomForestEvaluator.h
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(sources FILES ${sources})
SOURCE_GROUP(headers FILES ${headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/evaluation/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/rafl/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetAppTarget.cmake)

#################################
# Specify the libraries to link #
#################################

TARGET_LINK_LIBRARIES(${targetname} evaluation rafl tvgutil)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake)

#############################
# Specify things to install #
#############################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/InstallApp.cmake)
