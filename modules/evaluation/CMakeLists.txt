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
SET(toplevel_headers
include/evaluation/AlgorithmEvaluator.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${splitgenerators_sources}
)

SET(headers
${splitgenerators_headers}
${toplevel_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${toplevel_headers})
SOURCE_GROUP(splitgenerators FILES ${splitgenerators_sources} ${splitgenerators_headers})

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
