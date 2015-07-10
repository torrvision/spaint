#############################################
# CMakeLists.txt for modules/raflevaluation #
#############################################

###########################
# Specify the target name #
###########################

SET(targetname raflevaluation)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)

#############################
# Specify the project files #
#############################

##
SET(toplevel_sources
src/Dummy.cpp
)

SET(toplevel_headers
include/raflevaluation/RandomForestEvaluator.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${toplevel_sources}
)

SET(headers
${toplevel_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(toplevel FILES ${toplevel_sources} ${toplevel_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/raflevaluation)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/evaluation/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/rafl/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetLibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
