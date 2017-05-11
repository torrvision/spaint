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
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseLodePNG.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)

#############################
# Specify the project files #
#############################

##
SET(base_sources
src/base/MemoryBlockFactory.cpp
)

SET(base_headers
include/itmx/base/ITMImagePtrTypes.h
include/itmx/base/ITMMemoryBlockPtrTypes.h
include/itmx/base/ITMObjectPtrTypes.h
include/itmx/base/MemoryBlockFactory.h
)

##
SET(persistence_sources
src/persistence/ImagePersister.cpp
src/persistence/PosePersister.cpp
)

SET(persistence_headers
include/itmx/persistence/ImagePersister.h
include/itmx/persistence/PosePersister.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${base_sources}
${persistence_sources}
)

SET(headers
${base_headers}
${persistence_headers}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(base FILES ${base_sources} ${base_headers})
SOURCE_GROUP(persistence FILES ${persistence_sources} ${persistence_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/itmx)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
