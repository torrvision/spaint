#####################################
# CMakeLists.txt for modules/spaint #
#####################################

###########################
# Specify the target name #
###########################

SET(targetname spaint)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOVR.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseSDL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseVicon.cmake)

#############################
# Specify the project files #
#############################

##
SET(core_sources
src/core/SpaintInteractor.cpp
src/core/SpaintModel.cpp
src/core/SpaintPipeline.cpp
src/core/SpaintRaycaster.cpp
)

SET(core_headers
include/spaint/core/SpaintInteractor.h
include/spaint/core/SpaintModel.h
include/spaint/core/SpaintPipeline.h
include/spaint/core/SpaintRaycaster.h
)

##
SET(input_sources
src/input/InputState.cpp
)

SET(input_headers
include/spaint/input/InputState.h
include/spaint/input/MouseButton.h
)

##
SET(marking_cpu_sources
src/marking/cpu/VoxelMarker_CPU.cpp
)

SET(marking_cpu_headers
include/spaint/marking/cpu/VoxelMarker_CPU.h
)

##
SET(marking_cuda_sources
src/marking/cuda/VoxelMarker_CUDA.cu
)

SET(marking_cuda_headers
include/spaint/marking/cuda/VoxelMarker_CUDA.h
)

##
SET(marking_interface_headers
include/spaint/marking/interface/VoxelMarker.h
)

##
SET(marking_shared_headers
include/spaint/marking/shared/VoxelMarker_Shared.h
)

##
SET(ogl_sources
src/ogl/QuadricRenderer.cpp
)

SET(ogl_headers
include/spaint/ogl/QuadricRenderer.h
include/spaint/ogl/WrappedGL.h
)

##
SET(selection_transformers_cpu_sources
src/selection/transformers/cpu/VoxelToCubeSelectionTransformer_CPU.cpp
)

SET(selection_transformers_cpu_headers
include/spaint/selection/transformers/cpu/VoxelToCubeSelectionTransformer_CPU.h
)

##
SET(selection_transformers_cuda_sources
src/selection/transformers/cuda/VoxelToCubeSelectionTransformer_CUDA.cu
)

SET(selection_transformers_cuda_headers
include/spaint/selection/transformers/cuda/VoxelToCubeSelectionTransformer_CUDA.h
)

##
SET(selection_transformers_interface_headers
include/spaint/selection/transformers/interface/SelectionTransformer.h
include/spaint/selection/transformers/interface/VoxelToCubeSelectionTransformer.h
)

##
SET(selection_transformers_shared_headers
include/spaint/selection/transformers/shared/VoxelToCubeSelectionTransformer_Shared.h
)

##
SET(trackers_sources)
SET(trackers_headers)

IF(WITH_VICON)
  SET(trackers_sources ${trackers_sources} src/trackers/ViconTracker.cpp)
  SET(trackers_headers ${trackers_headers} include/spaint/trackers/ViconTracker.h)
ENDIF()

##
SET(util_sources
src/util/CameraPoseConverter.cpp
)

SET(util_headers
include/spaint/util/CameraPoseConverter.h
include/spaint/util/SpaintVoxel.h
)

##
SET(visualisers_cpu_sources
src/visualisers/cpu/SemanticVisualiser_CPU.cpp
)

SET(visualisers_cpu_headers
include/spaint/visualisers/cpu/SemanticVisualiser_CPU.h
)

##
SET(visualisers_cuda_sources
src/visualisers/cuda/SemanticVisualiser_CUDA.cu
)

SET(visualisers_cuda_headers
include/spaint/visualisers/cuda/SemanticVisualiser_CUDA.h
)

##
SET(visualisers_interface_headers
include/spaint/visualisers/interface/SemanticVisualiser.h
)

##
SET(visualisers_shared_headers
include/spaint/visualisers/shared/SemanticVisualiser_Shared.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${core_sources}
${input_sources}
${marking_cpu_sources}
${ogl_sources}
${selection_transformers_cpu_sources}
${trackers_sources}
${util_sources}
${visualisers_cpu_sources}
${visualisers_sources}
)

SET(headers
${core_headers}
${input_headers}
${marking_cpu_headers}
${marking_interface_headers}
${marking_shared_headers}
${ogl_headers}
${selection_transformers_cpu_headers}
${selection_transformers_interface_headers}
${selection_transformers_shared_headers}
${trackers_headers}
${util_headers}
${visualisers_cpu_headers}
${visualisers_interface_headers}
${visualisers_shared_headers}
)

IF(WITH_CUDA)
  SET(sources ${sources}
    ${marking_cuda_sources}
    ${selection_transformers_cuda_sources}
    ${visualisers_cuda_sources}
  )

  SET(headers ${headers}
    ${marking_cuda_headers}
    ${selection_transformers_cuda_headers}
    ${visualisers_cuda_headers}
  )
ENDIF()

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(core FILES ${core_sources} ${core_headers})
SOURCE_GROUP(input FILES ${input_sources} ${input_headers})
SOURCE_GROUP(marking\\cpu FILES ${marking_cpu_sources} ${marking_cpu_headers})
SOURCE_GROUP(marking\\cuda FILES ${marking_cuda_sources} ${marking_cuda_headers})
SOURCE_GROUP(marking\\interface FILES ${marking_interface_headers})
SOURCE_GROUP(marking\\shared FILES ${marking_shared_headers})
SOURCE_GROUP(ogl FILES ${ogl_sources} ${ogl_headers})
SOURCE_GROUP(selection\\transformers\\cpu FILES ${selection_transformers_cpu_sources} ${selection_transformers_cpu_headers})
SOURCE_GROUP(selection\\transformers\\cuda FILES ${selection_transformers_cuda_sources} ${selection_transformers_cuda_headers})
SOURCE_GROUP(selection\\transformers\\interface FILES ${selection_transformers_interface_headers})
SOURCE_GROUP(selection\\transformers\\shared FILES ${selection_transformers_shared_headers})
SOURCE_GROUP(trackers FILES ${trackers_sources} ${trackers_headers})
SOURCE_GROUP(util FILES ${util_sources} ${util_headers})
SOURCE_GROUP(visualisers\\cpu FILES ${visualisers_cpu_sources} ${visualisers_cpu_headers})
SOURCE_GROUP(visualisers\\cuda FILES ${visualisers_cuda_sources} ${visualisers_cuda_headers})
SOURCE_GROUP(visualisers\\interface FILES ${visualisers_interface_headers})
SOURCE_GROUP(visualisers\\shared FILES ${visualisers_shared_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/spaint)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/rigging/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)
#INCLUDE_DIRECTORIES(SYSTEM ${PROJECT_SOURCE_DIR}/helpers)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
