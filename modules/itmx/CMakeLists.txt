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
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
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
include/itmx/base/Settings.h
)

##
SET(geometry_sources
src/geometry/GeometryUtil.cpp
)

SET(geometry_headers
include/itmx/geometry/DualNumber.h
include/itmx/geometry/DualQuaternion.h
include/itmx/geometry/GeometryUtil.h
include/itmx/geometry/Screw.h
)

##
SET(imagesources_sources
src/imagesources/AsyncImageSourceEngine.cpp
src/imagesources/SingleRGBDImagePipe.cpp
)

SET(imagesources_headers
include/itmx/imagesources/AsyncImageSourceEngine.h
include/itmx/imagesources/SingleRGBDImagePipe.h
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

##
SET(relocalisation_sources
src/relocalisation/FernRelocaliser.cpp
src/relocalisation/RefiningRelocaliser.cpp
src/relocalisation/Relocaliser.cpp
)

SET(relocalisation_headers
include/itmx/relocalisation/FernRelocaliser.h
include/itmx/relocalisation/ICPRefiningRelocaliser.h
include/itmx/relocalisation/RefiningRelocaliser.h
include/itmx/relocalisation/Relocaliser.h
)

SET(relocalisation_templates
include/itmx/relocalisation/ICPRefiningRelocaliser.tpp
)

##
SET(util_sources
src/util/CameraPoseConverter.cpp
src/util/RGBDUtil.cpp
)

SET(util_headers
include/itmx/util/CameraPoseConverter.h
include/itmx/util/ColourConversion_Shared.h
include/itmx/util/RGBDUtil.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${base_sources}
${geometry_sources}
${imagesources_sources}
${persistence_sources}
${relocalisation_sources}
${util_sources}
)

SET(headers
${base_headers}
${geometry_headers}
${imagesources_headers}
${persistence_headers}
${relocalisation_headers}
${util_headers}
)

SET(templates
${relocalisation_templates}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(base FILES ${base_sources} ${base_headers})
SOURCE_GROUP(geometry FILES ${geometry_sources} ${geometry_headers})
SOURCE_GROUP(imagesources FILES ${imagesources_sources} ${imagesources_headers})
SOURCE_GROUP(persistence FILES ${persistence_sources} ${persistence_headers})
SOURCE_GROUP(relocalisation FILES ${relocalisation_sources} ${relocalisation_headers} ${relocalisation_templates})
SOURCE_GROUP(util FILES ${util_sources} ${util_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/itmx)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/rigging/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
