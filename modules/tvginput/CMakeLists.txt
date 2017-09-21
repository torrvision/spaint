#######################################
# CMakeLists.txt for modules/tvginput #
#######################################

###########################
# Specify the target name #
###########################

SET(targetname tvginput)

################################
# Specify the libraries to use #
################################

# None needed

#############################
# Specify the project files #
#############################

##
SET(toplevel_sources
src/InputState.cpp
)

SET(toplevel_headers
include/tvginput/InputState.h
include/tvginput/Joystick.h
include/tvginput/Keycode.h
include/tvginput/MouseButton.h
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

INCLUDE_DIRECTORIES(include/tvginput)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetLibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
