######################################
# CMakeLists.txt for modules/tvgutil #
######################################

###########################
# Specify the target name #
###########################

SET(targetname tvgutil)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)

#############################
# Specify the project files #
#############################

##
SET(toplevel_sources
src/IDAllocator.cpp
src/RandomNumberGenerator.cpp
)

SET(toplevel_headers
include/tvgutil/IDAllocator.h
include/tvgutil/LimitedContainer.h
include/tvgutil/PriorityQueue.h
include/tvgutil/PropertyUtil.h
include/tvgutil/RandomNumberGenerator.h
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

SOURCE_GROUP("" FILES ${toplevel_sources} ${toplevel_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/tvgutil)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetLibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
