#####################################
# CMakeLists.txt for apps/relocgui #
#####################################

###########################
# Specify the target name #
###########################

SET(targetname relocgui)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseGrove.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)

############################
# Check build requirements #
############################

IF(NOT WITH_OPENCV)
  MESSAGE(FATAL_ERROR "relocgui needs WITH_OPENCV be enabled.")
ENDIF()

#############################
# Specify the project files #
#############################

##
SET(toplevel_sources
  main.cpp
  RelocaliserApplication.cpp
)

SET(toplevel_headers
  RelocaliserApplication.h
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

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/itmx/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvginput/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)

#################################
# Specify the libraries to link #
#################################

# Note: spaint needs to precede rafl on Linux.
TARGET_LINK_LIBRARIES(${targetname} itmx tvginput tvgutil)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkGrove.cmake)

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenCV.cmake)

#########################################
# Copy resource files to the build tree #
#########################################

ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/apps/relocgui/resources" "$<TARGET_FILE_DIR:${targetname}>/resources")

IF(MSVC_IDE)
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/apps/relocgui/resources" "${PROJECT_BINARY_DIR}/apps/relocgui/resources")
ENDIF()

#############################
# Specify things to install #
#############################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/InstallApp.cmake)
INSTALL(DIRECTORY "${PROJECT_SOURCE_DIR}/apps/relocgui/resources" DESTINATION "bin/apps/relocgui")
