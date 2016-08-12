######################################
# CMakeLists.txt for unit/evaluation #
######################################

###############################
# Specify the test suite name #
###############################

SET(suitename evaluation)

##########################
# Specify the test names #
##########################

SET(testnames
CartesianProductParameterSetGenerator
ConfusionMatrixUtil
CoordinateDescentParameterSetGenerator
CrossValidationSplitGenerator
PerformanceMeasureUtil
RandomPermutationAndDivisionSplitGenerator
)

FOREACH(testname ${testnames})

SET(targetname "unittest_${suitename}_${testname}")

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)

#############################
# Specify the project files #
#############################

SET(sources
test_${testname}.cpp
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(sources FILES ${sources})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/evaluation/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetUnitTestTarget.cmake)

#################################
# Specify the libraries to link #
#################################

TARGET_LINK_LIBRARIES(${targetname} evaluation tvgutil)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)

ENDFOREACH()
