#####################################
# CMakeLists.txt for modules/spaint #
#####################################

###########################
# Specify the target name #
###########################

SET(targetname spaint)

###########################
# Offer low-power support #
###########################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/OfferLowPowerSupport.cmake)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseArrayFire.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseGLEW.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseLeap.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOVR.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseSDL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseTBB.cmake)
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
SET(features_sources
src/features/FeatureCalculatorFactory.cpp
)

SET(features_headers
include/spaint/features/FeatureCalculatorFactory.h
)

##
SET(features_cpu_sources
src/features/cpu/VOPFeatureCalculator_CPU.cpp
)

SET(features_cpu_headers
include/spaint/features/cpu/VOPFeatureCalculator_CPU.h
)

##
SET(features_cuda_sources
src/features/cuda/VOPFeatureCalculator_CUDA.cu
)

SET(features_cuda_headers
include/spaint/features/cuda/VOPFeatureCalculator_CUDA.h
)

##
SET(features_interface_sources
src/features/interface/VOPFeatureCalculator.cpp
)

SET(features_interface_headers
include/spaint/features/interface/FeatureCalculator.h
include/spaint/features/interface/VOPFeatureCalculator.h
)

##
SET(features_shared_headers
include/spaint/features/shared/VOPFeatureCalculator_Shared.h
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
SET(imageprocessing_cpu_sources
src/imageprocessing/cpu/ImageProcessor_CPU.cpp
)

SET(imageprocessing_cpu_headers
include/spaint/imageprocessing/cpu/ImageProcessor_CPU.h
)

##
SET(imageprocessing_cuda_sources
src/imageprocessing/cuda/ImageProcessor_CUDA.cu
)

SET(imageprocessing_cuda_headers
include/spaint/imageprocessing/cuda/ImageProcessor_CUDA.h
)

##
SET(imageprocessing_interface_sources
src/imageprocessing/interface/ImageProcessor.cpp
)

SET(imageprocessing_interface_headers
include/spaint/imageprocessing/interface/ImageProcessor.h
)

##
SET(imageprocessing_shared_headers
include/spaint/imageprocessing/shared/ImageProcessor_Shared.h
)

##
SET(markers_cpu_sources
src/markers/cpu/VoxelMarker_CPU.cpp
)

SET(markers_cpu_headers
include/spaint/markers/cpu/VoxelMarker_CPU.h
)

##
SET(markers_cuda_sources
src/markers/cuda/VoxelMarker_CUDA.cu
)

SET(markers_cuda_headers
include/spaint/markers/cuda/VoxelMarker_CUDA.h
)

##
SET(markers_interface_headers
include/spaint/markers/interface/VoxelMarker.h
)

##
SET(markers_shared_headers
include/spaint/markers/shared/VoxelMarker_Shared.h
)

##
SET(ocv_sources
src/ocv/OpenCVUtil.cpp
)

SET(ocv_headers
include/spaint/ocv/OpenCVUtil.h
)

##
SET(ogl_sources
src/ogl/FrameBuffer.cpp
src/ogl/QuadricRenderer.cpp
)

SET(ogl_headers
include/spaint/ogl/FrameBuffer.h
include/spaint/ogl/QuadricRenderer.h
include/spaint/ogl/WrappedGL.h
)

##
SET(picking_cpu_sources
src/picking/cpu/Picker_CPU.cpp
)

SET(picking_cpu_headers
include/spaint/picking/cpu/Picker_CPU.h
)

##
SET(picking_cuda_sources
src/picking/cuda/Picker_CUDA.cu
)

SET(picking_cuda_headers
include/spaint/picking/cuda/Picker_CUDA.h
)

##
SET(picking_interface_headers
include/spaint/picking/interface/Picker.h
)

##
SET(picking_shared_headers
include/spaint/picking/shared/Picker_Shared.h
)

##
SET(randomforest_sources
src/randomforest/ForestUtil.cpp
src/randomforest/SpaintDecisionFunctionGenerator.cpp
)

SET(randomforest_headers
include/spaint/randomforest/ForestUtil.h
include/spaint/randomforest/SpaintDecisionFunctionGenerator.h
)

##
SET(sampling_sources
src/sampling/VoxelSamplerFactory.cpp
)

SET(sampling_headers
include/spaint/sampling/VoxelSamplerFactory.h
)

##
SET(sampling_cpu_sources
src/sampling/cpu/PerLabelVoxelSampler_CPU.cpp
src/sampling/cpu/UniformVoxelSampler_CPU.cpp
)

SET(sampling_cpu_headers
include/spaint/sampling/cpu/PerLabelVoxelSampler_CPU.h
include/spaint/sampling/cpu/UniformVoxelSampler_CPU.h
)

##
SET(sampling_cuda_sources
src/sampling/cuda/PerLabelVoxelSampler_CUDA.cu
src/sampling/cuda/UniformVoxelSampler_CUDA.cu
)

SET(sampling_cuda_headers
include/spaint/sampling/cuda/PerLabelVoxelSampler_CUDA.h
include/spaint/sampling/cuda/UniformVoxelSampler_CUDA.h
)

##
SET(sampling_interface_sources
src/sampling/interface/PerLabelVoxelSampler.cpp
src/sampling/interface/UniformVoxelSampler.cpp
)

SET(sampling_interface_headers
include/spaint/sampling/interface/PerLabelVoxelSampler.h
include/spaint/sampling/interface/UniformVoxelSampler.h
)

##
SET(sampling_shared_headers
include/spaint/sampling/shared/PerLabelVoxelSampler_Shared.h
include/spaint/sampling/shared/UniformVoxelSampler_Shared.h
)

##
SET(selectiontransformers_sources
src/selectiontransformers/SelectionTransformerFactory.cpp
)

SET(selectiontransformers_headers
include/spaint/selectiontransformers/SelectionTransformerFactory.h
)

##
SET(selectiontransformers_cpu_sources
src/selectiontransformers/cpu/VoxelToCubeSelectionTransformer_CPU.cpp
)

SET(selectiontransformers_cpu_headers
include/spaint/selectiontransformers/cpu/VoxelToCubeSelectionTransformer_CPU.h
)

##
SET(selectiontransformers_cuda_sources
src/selectiontransformers/cuda/VoxelToCubeSelectionTransformer_CUDA.cu
)

SET(selectiontransformers_cuda_headers
include/spaint/selectiontransformers/cuda/VoxelToCubeSelectionTransformer_CUDA.h
)

##
SET(selectiontransformers_interface_sources
src/selectiontransformers/interface/SelectionTransformer.cpp
src/selectiontransformers/interface/SelectionTransformerVisitor.cpp
src/selectiontransformers/interface/VoxelToCubeSelectionTransformer.cpp
)

SET(selectiontransformers_interface_headers
include/spaint/selectiontransformers/interface/SelectionTransformer.h
include/spaint/selectiontransformers/interface/SelectionTransformerVisitor.h
include/spaint/selectiontransformers/interface/VoxelToCubeSelectionTransformer.h
)

##
SET(selectiontransformers_shared_headers
include/spaint/selectiontransformers/shared/VoxelToCubeSelectionTransformer_Shared.h
)

##
SET(selectors_sources
src/selectors/NullSelector.cpp
src/selectors/PickingSelector.cpp
src/selectors/SelectorVisitor.cpp
)

SET(selectors_headers
include/spaint/selectors/NullSelector.h
include/spaint/selectors/PickingSelector.h
include/spaint/selectors/Selector.h
include/spaint/selectors/SelectorVisitor.h
)

IF(WITH_LEAP)
  SET(selectors_sources ${selectors_sources} src/selectors/LeapSelector.cpp)
  SET(selectors_headers ${selectors_headers} include/spaint/selectors/LeapSelector.h)
ENDIF()

IF(WITH_ARRAYFIRE)
  SET(selectors_sources ${selectors_sources} src/selectors/TouchSelector.cpp)
  SET(selectors_headers ${selectors_headers} include/spaint/selectors/TouchSelector.h)
ENDIF()

##
SET(touch_sources
src/touch/TouchDetector.cpp
)

SET(touch_headers
include/spaint/touch/TouchDetector.h
)

##
SET(trackers_sources)
SET(trackers_headers)

IF(WITH_OVR)
  SET(trackers_sources ${trackers_sources} src/trackers/RiftTracker.cpp)
  SET(trackers_headers ${trackers_headers} include/spaint/trackers/RiftTracker.h)
ENDIF()

IF(WITH_VICON)
  SET(trackers_sources ${trackers_sources} src/trackers/ViconTracker.cpp)
  SET(trackers_headers ${trackers_headers} include/spaint/trackers/ViconTracker.h)
ENDIF()

##
SET(util_sources
src/util/CameraPoseConverter.cpp
src/util/LabelManager.cpp
src/util/MemoryBlockFactory.cpp
src/util/RGBDUtil.cpp
)

SET(util_headers
include/spaint/util/CameraPoseConverter.h
include/spaint/util/LabelManager.h
include/spaint/util/MemoryBlockFactory.h
include/spaint/util/RGBDUtil.h
include/spaint/util/SpaintVoxel.h
)

##
SET(visualisers_cpu_sources
src/visualisers/cpu/DepthVisualiser_CPU.cpp
src/visualisers/cpu/SemanticVisualiser_CPU.cpp
)

SET(visualisers_cpu_headers
include/spaint/visualisers/cpu/DepthVisualiser_CPU.h
include/spaint/visualisers/cpu/SemanticVisualiser_CPU.h
)

##
SET(visualisers_cuda_sources
src/visualisers/cuda/DepthVisualiser_CUDA.cu
src/visualisers/cuda/SemanticVisualiser_CUDA.cu
)

SET(visualisers_cuda_headers
include/spaint/visualisers/cuda/DepthVisualiser_CUDA.h
include/spaint/visualisers/cuda/SemanticVisualiser_CUDA.h
)

##
SET(visualisers_interface_sources
src/visualisers/interface/SemanticVisualiser.cpp
)

SET(visualisers_interface_headers
include/spaint/visualisers/interface/DepthVisualiser.h
include/spaint/visualisers/interface/SemanticVisualiser.h
)

##
SET(visualisers_shared_headers
include/spaint/visualisers/shared/DepthVisualiser_Shared.h
include/spaint/visualisers/shared/SemanticVisualiser_Shared.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${core_sources}
${features_sources}
${features_cpu_sources}
${features_interface_sources}
${input_sources}
${markers_cpu_sources}
${ogl_sources}
${picking_cpu_sources}
${randomforest_sources}
${sampling_sources}
${sampling_cpu_sources}
${sampling_interface_sources}
${selectiontransformers_sources}
${selectiontransformers_cpu_sources}
${selectiontransformers_interface_sources}
${selectors_sources}
${trackers_sources}
${util_sources}
${visualisers_cpu_sources}
${visualisers_interface_sources}
)

SET(headers
${core_headers}
${features_headers}
${features_cpu_headers}
${features_interface_headers}
${features_shared_headers}
${input_headers}
${markers_cpu_headers}
${markers_interface_headers}
${markers_shared_headers}
${ogl_headers}
${picking_cpu_headers}
${picking_interface_headers}
${picking_shared_headers}
${randomforest_headers}
${sampling_headers}
${sampling_cpu_headers}
${sampling_interface_headers}
${sampling_shared_headers}
${selectiontransformers_headers}
${selectiontransformers_cpu_headers}
${selectiontransformers_interface_headers}
${selectiontransformers_shared_headers}
${selectors_headers}
${trackers_headers}
${util_headers}
${visualisers_cpu_headers}
${visualisers_interface_headers}
${visualisers_shared_headers}
)

IF(WITH_ARRAYFIRE)
  SET(sources ${sources}
    ${imageprocessing_cpu_sources}
    ${imageprocessing_interface_sources}
    ${touch_sources}
  )
  SET(headers ${headers}
    ${imageprocessing_cpu_headers}
    ${imageprocessing_interface_headers}
    ${imageprocessing_shared_headers}
    ${touch_headers}
  )
ENDIF()

IF(WITH_CUDA)
  SET(sources ${sources}
    ${features_cuda_sources}
    ${markers_cuda_sources}
    ${picking_cuda_sources}
    ${sampling_cuda_sources}
    ${selectiontransformers_cuda_sources}
    ${visualisers_cuda_sources}
  )

  SET(headers ${headers}
    ${features_cuda_headers}
    ${markers_cuda_headers}
    ${picking_cuda_headers}
    ${sampling_cuda_headers}
    ${selectiontransformers_cuda_headers}
    ${visualisers_cuda_headers}
  )

  IF(WITH_ARRAYFIRE)
    SET(sources ${sources} ${imageprocessing_cuda_sources})
    SET(headers ${headers} ${imageprocessing_cuda_headers})
  ENDIF()
ENDIF()

IF(WITH_OPENCV)
  SET(sources ${sources} ${ocv_sources})
  SET(headers ${headers} ${ocv_headers})
ENDIF()

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(core FILES ${core_sources} ${core_headers})
SOURCE_GROUP(features FILES ${features_sources} ${features_headers})
SOURCE_GROUP(features\\cpu FILES ${features_cpu_sources} ${features_cpu_headers})
SOURCE_GROUP(features\\cuda FILES ${features_cuda_sources} ${features_cuda_headers})
SOURCE_GROUP(features\\interface FILES ${features_interface_sources} ${features_interface_headers})
SOURCE_GROUP(features\\shared FILES ${features_shared_headers})
SOURCE_GROUP(imageprocessing\\cpu FILES ${imageprocessing_cpu_sources} ${imageprocessing_cpu_headers})
SOURCE_GROUP(imageprocessing\\cuda FILES ${imageprocessing_cuda_sources} ${imageprocessing_cuda_headers})
SOURCE_GROUP(imageprocessing\\interface FILES ${imageprocessing_interface_sources} ${imageprocessing_interface_headers})
SOURCE_GROUP(imageprocessing\\shared FILES ${imageprocessing_shared_headers})
SOURCE_GROUP(input FILES ${input_sources} ${input_headers})
SOURCE_GROUP(markers\\cpu FILES ${markers_cpu_sources} ${markers_cpu_headers})
SOURCE_GROUP(markers\\cuda FILES ${markers_cuda_sources} ${markers_cuda_headers})
SOURCE_GROUP(markers\\interface FILES ${markers_interface_headers})
SOURCE_GROUP(markers\\shared FILES ${markers_shared_headers})
SOURCE_GROUP(ocv FILES ${ocv_sources} ${ocv_headers})
SOURCE_GROUP(ogl FILES ${ogl_sources} ${ogl_headers})
SOURCE_GROUP(picking\\cpu FILES ${picking_cpu_sources} ${picking_cpu_headers})
SOURCE_GROUP(picking\\cuda FILES ${picking_cuda_sources} ${picking_cuda_headers})
SOURCE_GROUP(picking\\interface FILES ${picking_interface_headers})
SOURCE_GROUP(picking\\shared FILES ${picking_shared_headers})
SOURCE_GROUP(randomforest FILES ${randomforest_sources} ${randomforest_headers})
SOURCE_GROUP(sampling FILES ${sampling_sources} ${sampling_headers})
SOURCE_GROUP(sampling\\cpu FILES ${sampling_cpu_sources} ${sampling_cpu_headers})
SOURCE_GROUP(sampling\\cuda FILES ${sampling_cuda_sources} ${sampling_cuda_headers})
SOURCE_GROUP(sampling\\interface FILES ${sampling_interface_sources} ${sampling_interface_headers})
SOURCE_GROUP(sampling\\shared FILES ${sampling_shared_headers})
SOURCE_GROUP(selectiontransformers FILES ${selectiontransformers_sources} ${selectiontransformers_headers})
SOURCE_GROUP(selectiontransformers\\cpu FILES ${selectiontransformers_cpu_sources} ${selectiontransformers_cpu_headers})
SOURCE_GROUP(selectiontransformers\\cuda FILES ${selectiontransformers_cuda_sources} ${selectiontransformers_cuda_headers})
SOURCE_GROUP(selectiontransformers\\interface FILES ${selectiontransformers_interface_sources} ${selectiontransformers_interface_headers})
SOURCE_GROUP(selectiontransformers\\shared FILES ${selectiontransformers_shared_headers})
SOURCE_GROUP(selectors FILES ${selectors_sources} ${selectors_headers})
SOURCE_GROUP(touch FILES ${touch_sources} ${touch_headers})
SOURCE_GROUP(trackers FILES ${trackers_sources} ${trackers_headers})
SOURCE_GROUP(util FILES ${util_sources} ${util_headers})
SOURCE_GROUP(visualisers\\cpu FILES ${visualisers_cpu_sources} ${visualisers_cpu_headers})
SOURCE_GROUP(visualisers\\cuda FILES ${visualisers_cuda_sources} ${visualisers_cuda_headers})
SOURCE_GROUP(visualisers\\interface FILES ${visualisers_interface_sources} ${visualisers_interface_headers})
SOURCE_GROUP(visualisers\\shared FILES ${visualisers_shared_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/spaint)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/rafl/include)
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
