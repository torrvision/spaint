##########################################
# CMakeLists.txt for apps/relocconverter #
##########################################

###########################
# Specify the target name #
###########################

SET(targetname relocconverter)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseScoreForests.cmake)

#############################
# Specify the project files #
#############################

##
SET(sources
main.cpp
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(sources FILES ${sources})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/spaint/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)
TARGET_LINK_LIBRARIES(${targetname} spaint tvgutil tvginput cudadevrt)

#################################
# Specify the libraries to link #
#################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkScoreForests.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenCV.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)

#############################
# Specify things to install #
#############################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/InstallApp.cmake)
