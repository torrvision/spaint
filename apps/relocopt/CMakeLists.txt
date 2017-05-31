####################################
# CMakeLists.txt for apps/relocopt #
####################################

###########################
# Specify the target name #
###########################

SET(targetname relocopt)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)

#############################
# Specify the project files #
#############################

##
SET(sources
main.cpp
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(sources FILES ${sources})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/evaluation/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetAppTarget.cmake)

#################################
# Specify the libraries to link #
#################################

TARGET_LINK_LIBRARIES(${targetname} evaluation tvgutil)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)

#########################################
# Copy resource files to the build tree #
#########################################

ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/apps/relocopt/resources" "$<TARGET_FILE_DIR:${targetname}>/resources")

IF(MSVC_IDE)
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/apps/relocopt/resources" "${PROJECT_BINARY_DIR}/apps/relocopt/resources")
ENDIF()

#############################
# Specify things to install #
#############################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/InstallApp.cmake)
INSTALL(DIRECTORY "${PROJECT_SOURCE_DIR}/apps/relocopt/resources" DESTINATION "bin/apps/relocopt" USE_SOURCE_PERMISSIONS)
