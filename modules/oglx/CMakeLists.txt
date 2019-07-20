###################################
# CMakeLists.txt for modules/oglx #
###################################

###########################
# Specify the target name #
###########################

SET(targetname oglx)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseGLEW.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseGLUT.cmake)

#############################
# Specify the project files #
#############################

##
SET(toplevel_sources
src/FrameBuffer.cpp
src/OpenGLUtil.cpp
src/QuadricRenderer.cpp
)

SET(toplevel_headers
include/oglx/FrameBuffer.h
include/oglx/OpenGLUtil.h
include/oglx/QuadricRenderer.h
include/oglx/WrappedGL.h
include/oglx/WrappedGLUT.h
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

INCLUDE_DIRECTORIES(include/oglx)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/rigging/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetLibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
