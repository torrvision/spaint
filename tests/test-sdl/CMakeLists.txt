#####################################
# CMakeLists.txt for tests/test-sdl #
#####################################

###########################
# Specify the target name #
###########################
set(TESTS "main" )
#set(TESTS "main" "02_getting_an_image_on_the_screen")
foreach(t ${TESTS})

#message(creating test: ${t})

SET(targetname test-sdl-${t})

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseSDL.cmake)

#############################
# Specify the project files #
#############################
#SET(sources main.cpp)
SET(sources ${t}.cpp)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(sources FILES ${sources})

##########################################
# Specify additional include directories #
##########################################

# None needed

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetTestTarget.cmake)

#################################
# Specify the libraries to link #
#################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkSDL.cmake)

endforeach()