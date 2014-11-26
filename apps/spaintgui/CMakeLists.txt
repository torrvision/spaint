#####################################
# CMakeLists.txt for apps/spaintgui #
#####################################

###########################
# Specify the target name #
###########################

SET(targetname spaintgui)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOVR.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseSDL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseVicon.cmake)

#############################
# Specify the project files #
#############################

##
SET(base_sources
main.cpp
Application.cpp
WindowedRenderer.cpp
)

IF(WITH_CUDA)
  SET(cuda_sources CUDAInstantiations.cu)
ELSE()
  SET(cuda_sources)
ENDIF()

IF(WITH_OVR)
  SET(ovr_sources RiftRenderer.cpp)
ELSE()
  SET(ovr_sources)
ENDIF()

SET(sources
${base_sources}
${cuda_sources}
${ovr_sources}
)

##
SET(base_headers
Application.h
Renderer.h
WindowedRenderer.h
)

IF(WITH_OVR)
  SET(ovr_headers RiftRenderer.h)
ELSE()
  SET(ovr_headers)
ENDIF()

SET(headers
${base_headers}
${ovr_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(sources FILES ${sources})
SOURCE_GROUP(headers FILES ${headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/rigging/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/spaint/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)

#################################
# Specify the libraries to link #
#################################

TARGET_LINK_LIBRARIES(${targetname} rigging spaint)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOVR.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkSDL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkVicon.cmake)

#############################
# Specify things to install #
#############################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/InstallApp.cmake)
