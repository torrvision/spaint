########################################
# CMakeLists.txt for modules/infermous #
########################################

###########################
# Specify the target name #
###########################

SET(targetname infermous)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)

#############################
# Specify the project files #
#############################

##
SET(base_headers
include/infermous/base/CRF2D.h
include/infermous/base/PairwisePotentialCalculator.h
)

##
SET(engines_headers
include/infermous/engines/MeanFieldInferenceEngine.h
)

##
SET(toplevel_sources
src/Dummy.cpp
)

SET(toplevel_headers
include/infermous/CRF2D.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${toplevel_sources}
)

SET(headers
${base_headers}
${engines_headers}
${toplevel_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${toplevel_sources} ${toplevel_headers})
SOURCE_GROUP(base FILES ${base_headers})
SOURCE_GROUP(engines FILES ${engines_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/infermous)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
