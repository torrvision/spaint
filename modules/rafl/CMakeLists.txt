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
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)

#############################
# Specify the project files #
#############################

##
SET(base_headers
include/rafl/base/Descriptor.h
)

##
SET(choppers_headers
include/rafl/choppers/CyclicTreeChopper.h
include/rafl/choppers/HeightLimitingTreeChopper.h
include/rafl/choppers/RandomTreeChopper.h
include/rafl/choppers/TimeBasedTreeChopper.h
include/rafl/choppers/TreeChopper.h
)

##
SET(core_headers
include/rafl/core/DecisionTree.h
include/rafl/core/RandomForest.h
)

##
SET(decisionfunctions_sources
src/decisionfunctions/FeatureThresholdingDecisionFunction.cpp
src/decisionfunctions/PairwiseOpAndThresholdDecisionFunction.cpp
)

SET(decisionfunctions_headers
include/rafl/decisionfunctions/CompositeDecisionFunctionGenerator.h
include/rafl/decisionfunctions/DecisionFunction.h
include/rafl/decisionfunctions/DecisionFunctionGenerator.h
include/rafl/decisionfunctions/DecisionFunctionGeneratorFactory.h
include/rafl/decisionfunctions/FeatureBasedDecisionFunctionGenerator.h
include/rafl/decisionfunctions/FeatureThresholdingDecisionFunction.h
include/rafl/decisionfunctions/FeatureThresholdingDecisionFunctionGenerator.h
include/rafl/decisionfunctions/PairwiseOpAndThresholdDecisionFunction.h
include/rafl/decisionfunctions/PairwiseOpAndThresholdDecisionFunctionGenerator.h
)

##
SET(examples_headers
include/rafl/examples/Example.h
include/rafl/examples/ExampleReservoir.h
include/rafl/examples/ExampleUtil.h
include/rafl/examples/UnitCircleExampleGenerator.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${decisionfunctions_sources}
)

SET(headers
${base_headers}
${choppers_headers}
${core_headers}
${decisionfunctions_headers}
${examples_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(base FILES ${base_headers})
SOURCE_GROUP(choppers FILES ${choppers_headers})
SOURCE_GROUP(core FILES ${core_headers})
SOURCE_GROUP(decisionfunctions FILES ${decisionfunctions_sources} ${decisionfunctions_headers})
SOURCE_GROUP(examples FILES ${examples_headers})

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
