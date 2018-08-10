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

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseALGLIB.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseArrayFire.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseGLEW.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseGraphviz.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseGrove.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseLeap.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseLodePNG.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseSDL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseTBB.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseVTK.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseZed.cmake)

#############################
# Specify the project files #
#############################

##
SET(collaboration_sources
src/collaboration/CollaborativePoseOptimiser.cpp
)

SET(collaboration_headers
include/spaint/collaboration/CollaborationMode.h
include/spaint/collaboration/CollaborativePoseOptimiser.h
include/spaint/collaboration/CollaborativeRelocalisation.h
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
SET(fiducials_sources
src/fiducials/AveragingFiducial.cpp
src/fiducials/Fiducial.cpp
src/fiducials/FiducialMeasurement.cpp
src/fiducials/FiducialPoseEstimator.cpp
src/fiducials/SimpleFiducial.cpp
)

SET(fiducials_headers
include/spaint/fiducials/AveragingFiducial.h
include/spaint/fiducials/Fiducial.h
include/spaint/fiducials/FiducialDetector.h
include/spaint/fiducials/FiducialMeasurement.h
include/spaint/fiducials/FiducialPoseEstimator.h
include/spaint/fiducials/SimpleFiducial.h
)

IF(WITH_OPENCV)
  SET(fiducials_sources ${fiducials_sources} src/fiducials/ArUcoFiducialDetector.cpp)
  SET(fiducials_headers ${fiducials_headers} include/spaint/fiducials/ArUcoFiducialDetector.h)
ENDIF()

##
SET(imageprocessing_sources)
SET(imageprocessing_headers)

IF(WITH_ARRAYFIRE)
  SET(imageprocessing_sources ${imageprocessing_sources}
    src/imageprocessing/ImageProcessorFactory.cpp
    src/imageprocessing/MedianFilterer.cpp
  )

  SET(imageprocessing_headers ${imageprocessing_headers}
    include/spaint/imageprocessing/ImageProcessorFactory.h
    include/spaint/imageprocessing/MedianFilterer.h
  )
ENDIF()

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
SET(markers_sources
src/markers/VoxelMarkerFactory.cpp
)

SET(markers_headers
include/spaint/markers/VoxelMarkerFactory.h
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
include/spaint/markers/shared/VoxelMarker_Settings.h
include/spaint/markers/shared/VoxelMarker_Shared.h
)

##
SET(ogl_sources
src/ogl/CameraRenderer.cpp
src/ogl/FrameBuffer.cpp
src/ogl/QuadricRenderer.cpp
)

SET(ogl_headers
include/spaint/ogl/CameraRenderer.h
include/spaint/ogl/FrameBuffer.h
include/spaint/ogl/QuadricRenderer.h
include/spaint/ogl/WrappedGL.h
include/spaint/ogl/WrappedGLUT.h
)

##
SET(pipelinecomponents_sources
src/pipelinecomponents/CollaborativeComponent.cpp
src/pipelinecomponents/CollaborativeContext.cpp
src/pipelinecomponents/ObjectSegmentationComponent.cpp
src/pipelinecomponents/ObjectSegmentationContext.cpp
src/pipelinecomponents/PropagationComponent.cpp
src/pipelinecomponents/SemanticSegmentationComponent.cpp
src/pipelinecomponents/SLAMComponent.cpp
src/pipelinecomponents/SLAMContext.cpp
src/pipelinecomponents/SmoothingComponent.cpp
)

SET(pipelinecomponents_headers
include/spaint/pipelinecomponents/CollaborativeComponent.h
include/spaint/pipelinecomponents/CollaborativeContext.h
include/spaint/pipelinecomponents/ObjectSegmentationComponent.h
include/spaint/pipelinecomponents/ObjectSegmentationContext.h
include/spaint/pipelinecomponents/PropagationComponent.h
include/spaint/pipelinecomponents/PropagationContext.h
include/spaint/pipelinecomponents/SemanticSegmentationComponent.h
include/spaint/pipelinecomponents/SemanticSegmentationContext.h
include/spaint/pipelinecomponents/SLAMComponent.h
include/spaint/pipelinecomponents/SLAMContext.h
include/spaint/pipelinecomponents/SmoothingComponent.h
include/spaint/pipelinecomponents/SmoothingContext.h
)

##
SET(propagation_sources
src/propagation/LabelPropagatorFactory.cpp
)

SET(propagation_headers
include/spaint/propagation/LabelPropagatorFactory.h
)

##
SET(propagation_cpu_sources
src/propagation/cpu/LabelPropagator_CPU.cpp
)

SET(propagation_cpu_headers
include/spaint/propagation/cpu/LabelPropagator_CPU.h
)

##
SET(propagation_cuda_sources
src/propagation/cuda/LabelPropagator_CUDA.cu
)

SET(propagation_cuda_headers
include/spaint/propagation/cuda/LabelPropagator_CUDA.h
)

##
SET(propagation_interface_sources
src/propagation/interface/LabelPropagator.cpp
)

SET(propagation_interface_headers
include/spaint/propagation/interface/LabelPropagator.h
)

##
SET(propagation_shared_headers
include/spaint/propagation/shared/LabelPropagator_Shared.h
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
SET(segmentation_sources
src/segmentation/ColourAppearanceModel.cpp
src/segmentation/SegmentationUtil.cpp
src/segmentation/Segmenter.cpp
)

SET(segmentation_headers
include/spaint/segmentation/ColourAppearanceModel.h
include/spaint/segmentation/SegmentationUtil.h
include/spaint/segmentation/Segmenter.h
)

IF(WITH_ARRAYFIRE AND WITH_OPENCV)
  SET(segmentation_sources ${segmentation_sources} src/segmentation/BackgroundSubtractingObjectSegmenter.cpp)
  SET(segmentation_headers ${segmentation_headers} include/spaint/segmentation/BackgroundSubtractingObjectSegmenter.h)
ENDIF()

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
SET(slamstate_sources
src/slamstate/SLAMState.cpp
)

SET(slamstate_headers
include/spaint/slamstate/SLAMState.h
)

##
SET(smoothing_sources
src/smoothing/LabelSmootherFactory.cpp
)

SET(smoothing_headers
include/spaint/smoothing/LabelSmootherFactory.h
)

##
SET(smoothing_cpu_sources
src/smoothing/cpu/LabelSmoother_CPU.cpp
)

SET(smoothing_cpu_headers
include/spaint/smoothing/cpu/LabelSmoother_CPU.h
)

##
SET(smoothing_cuda_sources
src/smoothing/cuda/LabelSmoother_CUDA.cu
)

SET(smoothing_cuda_headers
include/spaint/smoothing/cuda/LabelSmoother_CUDA.h
)

##
SET(smoothing_interface_sources
src/smoothing/interface/LabelSmoother.cpp
)

SET(smoothing_interface_headers
include/spaint/smoothing/interface/LabelSmoother.h
)

##
SET(smoothing_shared_headers
include/spaint/smoothing/shared/LabelSmoother_Shared.h
)

##
SET(touch_sources
src/touch/TouchDescriptorCalculator.cpp
src/touch/TouchDetector.cpp
src/touch/TouchSettings.cpp
)

SET(touch_headers
include/spaint/touch/TouchDescriptorCalculator.h
include/spaint/touch/TouchDetector.h
include/spaint/touch/TouchSettings.h
)

##
SET(util_sources
src/util/LabelManager.cpp
)

SET(util_headers
include/spaint/util/CameraFactory.h
include/spaint/util/LabelManager.h
include/spaint/util/SpaintSurfel.h
include/spaint/util/SpaintSurfelScene.h
include/spaint/util/SpaintVoxel.h
include/spaint/util/SpaintVoxelScene.h
)

##
SET(visualisation_sources
src/visualisation/SemanticVisualiserFactory.cpp
src/visualisation/VisualisationGenerator.cpp
)

SET(visualisation_headers
include/spaint/visualisation/SemanticVisualiserFactory.h
include/spaint/visualisation/VisualisationGenerator.h
)

##
SET(visualisation_cpu_sources
src/visualisation/cpu/SemanticVisualiser_CPU.cpp
)

SET(visualisation_cpu_headers
include/spaint/visualisation/cpu/SemanticVisualiser_CPU.h
)

##
SET(visualisation_cuda_sources
src/visualisation/cuda/SemanticVisualiser_CUDA.cu
)

SET(visualisation_cuda_headers
include/spaint/visualisation/cuda/SemanticVisualiser_CUDA.h
)

##
SET(visualisation_interface_sources
src/visualisation/interface/SemanticVisualiser.cpp
)

SET(visualisation_interface_headers
include/spaint/visualisation/interface/SemanticVisualiser.h
)

##
SET(visualisation_shared_headers
include/spaint/visualisation/shared/SemanticVisualiser_Settings.h
include/spaint/visualisation/shared/SemanticVisualiser_Shared.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${collaboration_sources}
${features_sources}
${features_cpu_sources}
${features_interface_sources}
${fiducials_sources}
${imageprocessing_sources}
${markers_sources}
${markers_cpu_sources}
${ogl_sources}
${pipelinecomponents_sources}
${propagation_sources}
${propagation_cpu_sources}
${propagation_interface_sources}
${randomforest_sources}
${sampling_sources}
${sampling_cpu_sources}
${sampling_interface_sources}
${segmentation_sources}
${selectiontransformers_sources}
${selectiontransformers_cpu_sources}
${selectiontransformers_interface_sources}
${selectors_sources}
${slamstate_sources}
${smoothing_sources}
${smoothing_cpu_sources}
${smoothing_interface_sources}
${util_sources}
${visualisation_sources}
${visualisation_cpu_sources}
${visualisation_interface_sources}
)

SET(headers
${collaboration_headers}
${features_headers}
${features_cpu_headers}
${features_interface_headers}
${features_shared_headers}
${fiducials_headers}
${imageprocessing_headers}
${markers_headers}
${markers_cpu_headers}
${markers_interface_headers}
${markers_shared_headers}
${ogl_headers}
${pipelinecomponents_headers}
${propagation_headers}
${propagation_cpu_headers}
${propagation_interface_headers}
${propagation_shared_headers}
${randomforest_headers}
${sampling_headers}
${sampling_cpu_headers}
${sampling_interface_headers}
${sampling_shared_headers}
${segmentation_headers}
${selectiontransformers_headers}
${selectiontransformers_cpu_headers}
${selectiontransformers_interface_headers}
${selectiontransformers_shared_headers}
${selectors_headers}
${slamstate_headers}
${smoothing_headers}
${smoothing_cpu_headers}
${smoothing_interface_headers}
${smoothing_shared_headers}
${util_headers}
${visualisation_headers}
${visualisation_cpu_headers}
${visualisation_interface_headers}
${visualisation_shared_headers}
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
    ${propagation_cuda_sources}
    ${sampling_cuda_sources}
    ${selectiontransformers_cuda_sources}
    ${smoothing_cuda_sources}
    ${visualisation_cuda_sources}
  )

  SET(headers ${headers}
    ${features_cuda_headers}
    ${markers_cuda_headers}
    ${propagation_cuda_headers}
    ${sampling_cuda_headers}
    ${selectiontransformers_cuda_headers}
    ${smoothing_cuda_headers}
    ${visualisation_cuda_headers}
  )

  IF(WITH_ARRAYFIRE)
    SET(sources ${sources} ${imageprocessing_cuda_sources})
    SET(headers ${headers} ${imageprocessing_cuda_headers})
  ENDIF()
ENDIF()

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(collaboration FILES ${collaboration_sources} ${collaboration_headers})
SOURCE_GROUP(features FILES ${features_sources} ${features_headers})
SOURCE_GROUP(features\\cpu FILES ${features_cpu_sources} ${features_cpu_headers})
SOURCE_GROUP(features\\cuda FILES ${features_cuda_sources} ${features_cuda_headers})
SOURCE_GROUP(features\\interface FILES ${features_interface_sources} ${features_interface_headers})
SOURCE_GROUP(features\\shared FILES ${features_shared_headers})
SOURCE_GROUP(fiducials FILES ${fiducials_sources} ${fiducials_headers})
SOURCE_GROUP(imageprocessing FILES ${imageprocessing_sources} ${imageprocessing_headers})
SOURCE_GROUP(imageprocessing\\cpu FILES ${imageprocessing_cpu_sources} ${imageprocessing_cpu_headers})
SOURCE_GROUP(imageprocessing\\cuda FILES ${imageprocessing_cuda_sources} ${imageprocessing_cuda_headers})
SOURCE_GROUP(imageprocessing\\interface FILES ${imageprocessing_interface_sources} ${imageprocessing_interface_headers})
SOURCE_GROUP(imageprocessing\\shared FILES ${imageprocessing_shared_headers})
SOURCE_GROUP(markers FILES ${markers_sources} ${markers_headers})
SOURCE_GROUP(markers\\cpu FILES ${markers_cpu_sources} ${markers_cpu_headers})
SOURCE_GROUP(markers\\cuda FILES ${markers_cuda_sources} ${markers_cuda_headers})
SOURCE_GROUP(markers\\interface FILES ${markers_interface_headers})
SOURCE_GROUP(markers\\shared FILES ${markers_shared_headers})
SOURCE_GROUP(ogl FILES ${ogl_sources} ${ogl_headers})
SOURCE_GROUP(pipelinecomponents FILES ${pipelinecomponents_sources} ${pipelinecomponents_headers})
SOURCE_GROUP(propagation FILES ${propagation_sources} ${propagation_headers})
SOURCE_GROUP(propagation\\cpu FILES ${propagation_cpu_sources} ${propagation_cpu_headers})
SOURCE_GROUP(propagation\\cuda FILES ${propagation_cuda_sources} ${propagation_cuda_headers})
SOURCE_GROUP(propagation\\interface FILES ${propagation_interface_sources} ${propagation_interface_headers})
SOURCE_GROUP(propagation\\shared FILES ${propagation_shared_headers})
SOURCE_GROUP(randomforest FILES ${randomforest_sources} ${randomforest_headers})
SOURCE_GROUP(sampling FILES ${sampling_sources} ${sampling_headers})
SOURCE_GROUP(sampling\\cpu FILES ${sampling_cpu_sources} ${sampling_cpu_headers})
SOURCE_GROUP(sampling\\cuda FILES ${sampling_cuda_sources} ${sampling_cuda_headers})
SOURCE_GROUP(sampling\\interface FILES ${sampling_interface_sources} ${sampling_interface_headers})
SOURCE_GROUP(sampling\\shared FILES ${sampling_shared_headers})
SOURCE_GROUP(segmentation FILES ${segmentation_sources} ${segmentation_headers})
SOURCE_GROUP(selectiontransformers FILES ${selectiontransformers_sources} ${selectiontransformers_headers})
SOURCE_GROUP(selectiontransformers\\cpu FILES ${selectiontransformers_cpu_sources} ${selectiontransformers_cpu_headers})
SOURCE_GROUP(selectiontransformers\\cuda FILES ${selectiontransformers_cuda_sources} ${selectiontransformers_cuda_headers})
SOURCE_GROUP(selectiontransformers\\interface FILES ${selectiontransformers_interface_sources} ${selectiontransformers_interface_headers})
SOURCE_GROUP(selectiontransformers\\shared FILES ${selectiontransformers_shared_headers})
SOURCE_GROUP(selectors FILES ${selectors_sources} ${selectors_headers})
SOURCE_GROUP(slamstate FILES ${slamstate_sources} ${slamstate_headers})
SOURCE_GROUP(smoothing FILES ${smoothing_sources} ${smoothing_headers})
SOURCE_GROUP(smoothing\\cpu FILES ${smoothing_cpu_sources} ${smoothing_cpu_headers})
SOURCE_GROUP(smoothing\\cuda FILES ${smoothing_cuda_sources} ${smoothing_cuda_headers})
SOURCE_GROUP(smoothing\\interface FILES ${smoothing_interface_sources} ${smoothing_interface_headers})
SOURCE_GROUP(smoothing\\shared FILES ${smoothing_shared_headers})
SOURCE_GROUP(touch FILES ${touch_sources} ${touch_headers})
SOURCE_GROUP(util FILES ${util_sources} ${util_headers})
SOURCE_GROUP(visualisation FILES ${visualisation_sources} ${visualisation_headers})
SOURCE_GROUP(visualisation\\cpu FILES ${visualisation_cpu_sources} ${visualisation_cpu_headers})
SOURCE_GROUP(visualisation\\cuda FILES ${visualisation_cuda_sources} ${visualisation_cuda_headers})
SOURCE_GROUP(visualisation\\interface FILES ${visualisation_interface_sources} ${visualisation_interface_headers})
SOURCE_GROUP(visualisation\\shared FILES ${visualisation_shared_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/spaint)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/helpers)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/itmx/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/orx/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/rafl/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/rigging/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvginput/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
