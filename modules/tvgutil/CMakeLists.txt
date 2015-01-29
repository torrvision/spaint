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
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)

#############################
# Specify the project files #
#############################

##
SET(colours_headers
include/tvgutil/colours/PaletteGenerator.h
include/tvgutil/colours/RGBAColour.h
)

##
SET(timers_headers
include/tvgutil/timers/AverageTimer.h
include/tvgutil/timers/Timer.h
)

##
SET(toplevel_sources
src/IDAllocator.cpp
src/PropertyUtil.cpp
src/RandomNumberGenerator.cpp
)

SET(toplevel_headers
include/tvgutil/ArgUtil.h
include/tvgutil/IDAllocator.h
include/tvgutil/LimitedContainer.h
include/tvgutil/MapUtil.h
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
${colours_headers}
${timers_headers}
${toplevel_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${toplevel_sources} ${toplevel_headers})
SOURCE_GROUP(colours FILES ${colours_headers})
SOURCE_GROUP(timers FILES ${timers_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/tvgutil)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
