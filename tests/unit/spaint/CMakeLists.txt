###################################
# CMakeLists.txt for unit/spaint #
###################################

###############################
# Specify the test suite name #
###############################

SET(suitename spaint)

##########################
# Specify the test names #
##########################

SET(testnames
ColourConversion
)

FOREACH(testname ${testnames})

SET(targetname "unittest_${suitename}_${testname}")

################################
# Specify the libraries to use #
################################

#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseArrayFire.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseGLEW.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseLeap.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseLodePNG.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOVR.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseSDL.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseTBB.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseVicon.cmake)

#############################
# Specify the project files #
#############################

SET(sources
test_${testname}.cpp
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(sources FILES ${sources})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/spaint/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetUnitTestTarget.cmake)

#################################
# Specify the libraries to link #
#################################

TARGET_LINK_LIBRARIES(${targetname} spaint)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)

ENDFOREACH()
