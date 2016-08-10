######################################
# CMakeLists.txt for modules/tvgplot #
######################################

###########################
# Specify the target name #
###########################

SET(targetname tvgplot)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)

#############################
# Specify the project files #
#############################

##
SET(sources
src/PaletteGenerator.cpp
src/PlotWindow.cpp
)

SET(headers
include/tvgplot/PaletteGenerator.h
include/tvgplot/PlotWindow.h
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${sources} ${headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/tvgplot)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetLibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
