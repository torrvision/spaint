###################################
# CMakeLists.txt for modules/rafl #
###################################

###########################
# Specify the target name #
###########################

SET(targetname rafl)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)

#############################
# Specify the project files #
#############################

##
SET(base_headers
include/rafl/base/DecisionFunction.h
include/rafl/base/Descriptor.h
include/rafl/base/Histogram.h
include/rafl/base/ProbabilityMassFunction.h
)

##
SET(examples_sources
src/examples/ExampleReservoir.cpp
)

SET(examples_headers
include/rafl/examples/Example.h
include/rafl/examples/ExampleReservoir.h
)

##
SET(toplevel_headers
include/rafl/DecisionTree.h
include/rafl/RandomForest.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${examples_sources}
)

SET(headers
${base_headers}
${examples_headers}
${toplevel_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${toplevel_headers})
SOURCE_GROUP(base FILES ${base_headers})
SOURCE_GROUP(examples FILES ${examples_sources} ${examples_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/rafl)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
