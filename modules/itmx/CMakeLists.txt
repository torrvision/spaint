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
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOVR.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseVicon.cmake)

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
src/imagesources/RemoteImageSourceEngine.cpp
src/imagesources/SingleRGBDImagePipe.cpp
)

SET(imagesources_headers
include/itmx/imagesources/AsyncImageSourceEngine.h
include/itmx/imagesources/RemoteImageSourceEngine.h
include/itmx/imagesources/SingleRGBDImagePipe.h
)

##
SET(ocv_sources
src/ocv/OpenCVUtil.cpp
)

SET(ocv_headers
include/itmx/ocv/OpenCVUtil.h
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
src/relocalisation/NullRelocaliser.cpp
src/relocalisation/RefiningRelocaliser.cpp
src/relocalisation/Relocaliser.cpp
)

SET(relocalisation_headers
include/itmx/relocalisation/FernRelocaliser.h
include/itmx/relocalisation/ICPRefiningRelocaliser.h
include/itmx/relocalisation/NullRelocaliser.h
include/itmx/relocalisation/RefiningRelocaliser.h
include/itmx/relocalisation/Relocaliser.h
)

SET(relocalisation_templates
include/itmx/relocalisation/ICPRefiningRelocaliser.tpp
)

##
SET(remotemapping_sources
src/remotemapping/AckMessage.cpp
src/remotemapping/BaseRGBDFrameMessage.cpp
src/remotemapping/CompressedRGBDFrameHeaderMessage.cpp
src/remotemapping/CompressedRGBDFrameMessage.cpp
src/remotemapping/MappingClient.cpp
src/remotemapping/MappingMessage.cpp
src/remotemapping/MappingServer.cpp
src/remotemapping/RGBDCalibrationMessage.cpp
src/remotemapping/RGBDFrameCompressor.cpp
src/remotemapping/RGBDFrameMessage.cpp
)

SET(remotemapping_headers
include/itmx/remotemapping/AckMessage.h
include/itmx/remotemapping/BaseRGBDFrameMessage.h
include/itmx/remotemapping/CompressedRGBDFrameHeaderMessage.h
include/itmx/remotemapping/CompressedRGBDFrameMessage.h
include/itmx/remotemapping/DepthCompressionType.h
include/itmx/remotemapping/MappingClient.h
include/itmx/remotemapping/MappingMessage.h
include/itmx/remotemapping/MappingServer.h
include/itmx/remotemapping/RGBCompressionType.h
include/itmx/remotemapping/RGBDCalibrationMessage.h
include/itmx/remotemapping/RGBDFrameCompressor.h
include/itmx/remotemapping/RGBDFrameMessage.h
)

##
SET(trackers_sources
src/trackers/RemoteTracker.cpp
src/trackers/TrackerFactory.cpp
)

SET(trackers_headers
include/itmx/trackers/FallibleTracker.h
include/itmx/trackers/RemoteTracker.h
include/itmx/trackers/TrackerFactory.h
)

IF(WITH_OVR)
  SET(trackers_sources ${trackers_sources} src/trackers/RiftTracker.cpp)
  SET(trackers_headers ${trackers_headers} include/itmx/trackers/RiftTracker.h)
ENDIF()

IF(WITH_VICON)
  SET(trackers_sources ${trackers_sources}
    src/trackers/RobustViconTracker.cpp
    src/trackers/ViconTracker.cpp
  )

  SET(trackers_headers ${trackers_headers}
    include/itmx/trackers/RobustViconTracker.h
    include/itmx/trackers/ViconTracker.h
  )
ENDIF()

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
${remotemapping_sources}
${trackers_sources}
${util_sources}
)

SET(headers
${base_headers}
${geometry_headers}
${imagesources_headers}
${persistence_headers}
${relocalisation_headers}
${remotemapping_headers}
${trackers_headers}
${util_headers}
)

SET(templates
${relocalisation_templates}
)

IF(WITH_OPENCV)
  SET(sources ${sources} ${ocv_sources})
  SET(headers ${headers} ${ocv_headers})
ENDIF()

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(base FILES ${base_sources} ${base_headers})
SOURCE_GROUP(geometry FILES ${geometry_sources} ${geometry_headers})
SOURCE_GROUP(imagesources FILES ${imagesources_sources} ${imagesources_headers})
SOURCE_GROUP(ocv FILES ${ocv_sources} ${ocv_headers})
SOURCE_GROUP(persistence FILES ${persistence_sources} ${persistence_headers})
SOURCE_GROUP(relocalisation FILES ${relocalisation_sources} ${relocalisation_headers} ${relocalisation_templates})
SOURCE_GROUP(remotemapping FILES ${remotemapping_sources} ${remotemapping_headers})
SOURCE_GROUP(trackers FILES ${trackers_sources} ${trackers_headers})
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
