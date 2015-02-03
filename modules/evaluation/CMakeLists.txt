#########################################
# CMakeLists.txt for modules/evaluation #
#########################################

###########################
# Specify the target name #
###########################

SET(targetname evaluation)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)

#############################
# Specify the project files #
#############################

##
SET(core_sources
src/core/PerformanceMeasure.cpp
src/core/PerformanceTable.cpp
)

SET(core_headers
include/evaluation/core/LearnerEvaluator.h
include/evaluation/core/PerformanceMeasure.h
include/evaluation/core/PerformanceTable.h
)

##
SET(splitgenerators_sources
src/splitgenerators/CrossValidationSplitGenerator.cpp
src/splitgenerators/RandomlyPermuteAndDivideSplitGenerator.cpp
)

SET(splitgenerators_headers
include/evaluation/splitgenerators/CrossValidationSplitGenerator.h
include/evaluation/splitgenerators/RandomlyPermuteAndDivideSplitGenerator.h
include/evaluation/splitgenerators/SplitGenerator.h
)

##
SET(util_sources
src/util/CartesianProductParameterSetGenerator.cpp
src/util/ConfusionMatrixUtil.cpp
)

SET(util_headers
include/evaluation/util/CartesianProductParameterSetGenerator.h
include/evaluation/util/ConfusionMatrixUtil.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${core_sources}
${splitgenerators_sources}
${util_sources}
)

SET(headers
${core_headers}
${splitgenerators_headers}
${util_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(core FILES ${core_sources} ${core_headers})
SOURCE_GROUP(splitgenerators FILES ${splitgenerators_sources} ${splitgenerators_headers})
SOURCE_GROUP(util FILES ${util_sources} ${util_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/evaluation)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetLibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
