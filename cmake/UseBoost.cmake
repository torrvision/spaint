##################
# UseBoost.cmake #
##################

SET(Boost_ADDITIONAL_VERSIONS "1.53" "1.53.0" "1.54" "1.54.0" "1.55" "1.55.0" "1.56" "1.56.0" "1.58.0")
SET(BOOST_ROOT "${PROJECT_SOURCE_DIR}/libraries/boost_1_58_0" CACHE FILEPATH "The Boost root directory")
SET(Boost_USE_STATIC_LIBS ON)
FIND_PACKAGE(Boost REQUIRED COMPONENTS timer chrono date_time filesystem program_options regex serialization system thread unit_test_framework)
IF(Boost_FOUND)
  INCLUDE_DIRECTORIES(SYSTEM ${Boost_INCLUDE_DIRS})
  LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
  ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
ELSE(Boost_FOUND)
  MESSAGE(FATAL_ERROR "Boost not found. Please set the directories manually using the advanced view in CMake.")
ENDIF(Boost_FOUND)
