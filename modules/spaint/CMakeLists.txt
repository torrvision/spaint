#####################################
# CMakeLists.txt for modules/spaint #
#####################################

###########################
# Specify the target name #
###########################

SET(targetname spaint)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOVR.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseSDL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseVicon.cmake)

#############################
# Specify the project files #
#############################

##
SET(input_sources
src/input/InputState.cpp
)

SET(input_headers
include/spaint/input/InputState.h
include/spaint/input/MouseButton.h
)

##
SET(ogl_headers
include/spaint/ogl/WrappedGL.h
)

##
SET(toplevel_sources
src/SpaintEngine.cpp
)

SET(toplevel_headers
include/spaint/SpaintEngine.h
)

##
SET(util_sources
src/util/CameraPoseConverter.cpp
)

SET(util_headers
include/spaint/util/CameraPoseConverter.h
include/spaint/util/SpaintVoxel.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${input_sources}
${toplevel_sources}
${util_sources}
)

SET(headers
${input_headers}
${ogl_headers}
${toplevel_headers}
${util_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${toplevel_sources} ${toplevel_headers})
SOURCE_GROUP(input FILES ${input_sources} ${input_headers})
SOURCE_GROUP(ogl FILES ${ogl_headers})
SOURCE_GROUP(util FILES ${util_sources} ${util_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/spaint)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/rigging/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)
#INCLUDE_DIRECTORIES(SYSTEM ${PROJECT_SOURCE_DIR}/helpers)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
