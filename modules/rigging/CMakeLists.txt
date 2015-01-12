######################################
# CMakeLists.txt for modules/rigging #
######################################

###########################
# Specify the target name #
###########################

SET(targetname rigging)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)

#############################
# Specify the project files #
#############################

##
SET(toplevel_sources
src/CompositeCamera.cpp
src/DerivedCamera.cpp
src/SimpleCamera.cpp
)

SET(toplevel_headers
include/rigging/Camera.h
include/rigging/CompositeCamera.h
include/rigging/DerivedCamera.h
include/rigging/MoveableCamera.h
include/rigging/SimpleCamera.h
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

INCLUDE_DIRECTORIES(include/rigging)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetLibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
