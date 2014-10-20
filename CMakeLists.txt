#######################################
# Top-level CMakeLists.txt for spaint #
#######################################

cmake_minimum_required(VERSION 2.8)

PROJECT(spaint)

##########################
# Specify subdirectories #
##########################

OPTION(BUILD_APPS "Build the applications?" ON)

IF(BUILD_APPS)
  ADD_SUBDIRECTORY(apps)
ENDIF(BUILD_APPS)

OPTION(BUILD_TESTS "Build the tests?" ON)

IF(BUILD_TESTS)
  ADD_SUBDIRECTORY(tests)
ENDIF(BUILD_TESTS)

###########################
# Specify the target name #
###########################

SET(targetname spaint)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseOpenCV.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseSDL.cmake)

#############################
# Specify the project files #
#############################

##
SET(input_sources
src/input/InputState.cpp
)

SET(input_headers
include/spaint/input/InputState.h
include/spaint/input/MouseButton.h
)

##
SET(main_sources
src/main/SpaintEngine.cpp
)

SET(main_headers
include/spaint/main/SpaintEngine.h
)

##
SET(ogl_headers
include/spaint/ogl/WrappedGL.h
)

##
SET(util_headers
include/spaint/util/SharedPtr.h
include/spaint/util/SpaintVoxel.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${input_sources}
${main_sources}
)

SET(headers
${input_headers}
${main_headers}
${ogl_headers}
${util_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(input FILES ${input_sources} ${input_headers})
SOURCE_GROUP(main FILES ${main_sources} ${main_headers})
SOURCE_GROUP(ogl FILES ${ogl_headers})
SOURCE_GROUP(util FILES ${util_headers})

#################################
# Add additional compiler flags #
#################################

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -ansi -pedantic")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/spaint)
#INCLUDE_DIRECTORIES(SYSTEM ${PROJECT_SOURCE_DIR}/helpers)

##########################################
# Specify the target and where to put it #
##########################################

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/lib)
CUDA_ADD_LIBRARY(${targetname} STATIC ${sources} ${headers} ${templates} OPTIONS --generate-code arch=compute_20,code=sm_20 --generate-code arch=compute_30,code=sm_30)

SET_TARGET_PROPERTIES(${targetname} PROPERTIES DEBUG_OUTPUT_NAME "spaint_d")

#########################################
# Specify the default install directory #
#########################################

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX ../install CACHE PATH "" FORCE)
ENDIF()

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
