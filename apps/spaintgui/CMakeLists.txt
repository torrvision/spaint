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

INCLUDE(${PROJECT_SOURCE_DIR}/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseOVR.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/UseSDL.cmake)

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

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/SetCUDAAppTarget.cmake)

#################################
# Specify the libraries to link #
#################################

TARGET_LINK_LIBRARIES(${targetname} spaint)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkOVR.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/LinkSDL.cmake)

#############################
# Specify things to install #
#############################

INCLUDE(${PROJECT_SOURCE_DIR}/InstallApp.cmake)
