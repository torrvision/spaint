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
src/core/ParamSetUtil.cpp
src/core/PerformanceMeasure.cpp
src/core/PerformanceMeasureUtil.cpp
src/core/PerformanceTable.cpp
)

SET(core_headers
include/evaluation/core/LearnerEvaluator.h
include/evaluation/core/ParamSetUtil.h
include/evaluation/core/PerformanceMeasure.h
include/evaluation/core/PerformanceMeasureUtil.h
include/evaluation/core/PerformanceResult.h
include/evaluation/core/PerformanceTable.h
)

##
SET(paramsetgenerators_sources
src/paramsetgenerators/CartesianProductParameterSetGenerator.cpp
src/paramsetgenerators/CoordinateDescentParameterSetGenerator.cpp
src/paramsetgenerators/ParameterSetGenerator.cpp
)

SET(paramsetgenerators_headers
include/evaluation/paramsetgenerators/CartesianProductParameterSetGenerator.h
include/evaluation/paramsetgenerators/CoordinateDescentParameterSetGenerator.h
include/evaluation/paramsetgenerators/ParameterSetGenerator.h
)

##
SET(splitgenerators_sources
src/splitgenerators/CrossValidationSplitGenerator.cpp
src/splitgenerators/RandomPermutationAndDivisionSplitGenerator.cpp
src/splitgenerators/RNGFunctor.cpp
)

SET(splitgenerators_headers
include/evaluation/splitgenerators/CrossValidationSplitGenerator.h
include/evaluation/splitgenerators/RandomPermutationAndDivisionSplitGenerator.h
include/evaluation/splitgenerators/RNGFunctor.h
include/evaluation/splitgenerators/SplitGenerator.h
)

##
SET(util_sources
src/util/ConfusionMatrixUtil.cpp
)

SET(util_headers
include/evaluation/util/ConfusionMatrixUtil.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${core_sources}
${paramsetgenerators_sources}
${splitgenerators_sources}
${util_sources}
)

SET(headers
${core_headers}
${paramsetgenerators_headers}
${splitgenerators_headers}
${util_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(core FILES ${core_sources} ${core_headers})
SOURCE_GROUP(paramsetgenerators FILES ${paramsetgenerators_sources} ${paramsetgenerators_headers})
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
