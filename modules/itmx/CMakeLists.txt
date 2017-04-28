###################################
# CMakeLists.txt for modules/itmx #
###################################

###########################
# Specify the target name #
###########################

SET(targetname itmx)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)

#############################
# Specify the project files #
#############################

##
SET(relocalisation_sources
src/relocalisation/FernRelocaliser.cpp
src/relocalisation/Relocaliser.cpp
src/relocalisation/RelocaliserFactory.cpp
)

SET(relocalisation_headers
include/itmx/relocalisation/FernRelocaliser.h
include/itmx/relocalisation/Relocaliser.h
include/itmx/relocalisation/RelocaliserFactory.h
)

##
SET(toplevel_sources
src/MemoryBlockFactory.cpp
)

SET(toplevel_headers
include/itmx/ITMImagePtrTypes.h
include/itmx/ITMMemoryBlockPtrTypes.h
include/itmx/ITMObjectPtrTypes.h
include/itmx/MemoryBlockFactory.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${relocalisation_sources}
${toplevel_sources}
)

SET(headers
${relocalisation_headers}
${toplevel_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${toplevel_sources} ${toplevel_headers})
SOURCE_GROUP("relocalisation" FILES ${relocalisation_sources} ${relocalisation_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/itmx)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
