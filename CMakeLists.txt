#######################################
# Top-level CMakeLists.txt for spaint #
#######################################

cmake_minimum_required(VERSION 2.8)

PROJECT(spaint)

#################################
# Add additional compiler flags #
#################################

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -ansi -pedantic")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

# If on Mac OS X 10.9 (Mavericks), disable the annoying deprecation warnings (e.g. for GLU).
IF(${CMAKE_SYSTEM} MATCHES "Darwin-13.")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
ENDIF()

#########################################
# Specify the default install directory #
#########################################

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX ../install CACHE PATH "" FORCE)
ENDIF()

##########################
# Specify subdirectories #
##########################

ADD_SUBDIRECTORY(modules)

OPTION(BUILD_APPS "Build the applications?" ON)

IF(BUILD_APPS)
  ADD_SUBDIRECTORY(apps)
ENDIF(BUILD_APPS)

OPTION(BUILD_TESTS "Build the tests?" ON)

IF(BUILD_TESTS)
  ADD_SUBDIRECTORY(tests)
ENDIF(BUILD_TESTS)

OPTION(BUILD_UNITTESTS "Build the unit tests?" ON)

IF(BUILD_UNITTESTS)
  ENABLE_TESTING()
  ADD_SUBDIRECTORY(unittests)
ENDIF(BUILD_UNITTESTS)