/**
 * spaintgui: Pipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Pipeline.h"
using namespace rafl;
using namespace spaint;

#include <ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <ITMLib/Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h>
#include <ITMLib/Engines/Swapping/ITMSwappingEngineFactory.h>
#include <ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h>
#include <ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
#include <ITMLib/Trackers/ITMTrackerFactory.h>
using namespace InputSource;
using namespace ITMLib;
using namespace ORUtils;
using namespace RelocLib;

#include <spaint/features/FeatureCalculatorFactory.h>
#include <spaint/propagation/LabelPropagatorFactory.h>
#include <spaint/randomforest/ForestUtil.h>
#include <spaint/randomforest/SpaintDecisionFunctionGenerator.h>
#include <spaint/sampling/VoxelSamplerFactory.h>
#include <spaint/smoothing/LabelSmootherFactory.h>
#include <spaint/util/MemoryBlockFactory.h>

#ifdef WITH_OPENCV
#include <spaint/ocv/OpenCVUtil.h>
#endif

#ifdef WITH_OVR
#include <spaint/trackers/RiftTracker.h>
#endif

#ifdef WITH_VICON
#include <spaint/trackers/RobustViconTracker.h>
#include <spaint/trackers/ViconTracker.h>
#endif

#define DEBUGGING 1

//#################### CONSTRUCTORS ####################

Pipeline::Pipeline(const CompositeImageSourceEngine_Ptr& imageSourceEngine, const Settings_Ptr& settings, const std::string& resourcesDir,
                   TrackerType trackerType, const std::string& trackerParams)
{
  m_state.m_imageSourceEngine = imageSourceEngine;
  m_state.m_resourcesDir = resourcesDir;
  m_state.m_trackerParams = trackerParams;
  m_state.m_trackerType = trackerType;
  initialise(settings);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool Pipeline::get_fusion_enabled() const
{
  return m_state.m_fusionEnabled;
}

ITMShortImage_Ptr Pipeline::get_input_raw_depth_image_copy() const
{
  ITMShortImage_Ptr copy(new ITMShortImage(m_state.m_inputRawDepthImage->noDims, true, false));
  copy->SetFrom(m_state.m_inputRawDepthImage.get(), ORUtils::MemoryBlock<short>::CPU_TO_CPU);
  return copy;
}

ITMUChar4Image_Ptr Pipeline::get_input_rgb_image_copy() const
{
  ITMUChar4Image_Ptr copy(new ITMUChar4Image(m_state.m_inputRGBImage->noDims, true, false));
  copy->SetFrom(m_state.m_inputRGBImage.get(), ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
  return copy;
}

const Interactor_Ptr& Pipeline::get_interactor()
{
  return m_state.m_interactor;
}

PipelineMode Pipeline::get_mode() const
{
  return m_state.m_mode;
}

const Model_Ptr& Pipeline::get_model()
{
  return m_state.m_model;
}

Model_CPtr Pipeline::get_model() const
{
  return m_state.m_model;
}

const Raycaster_Ptr& Pipeline::get_raycaster()
{
  return m_state.m_raycaster;
}

Raycaster_CPtr Pipeline::get_raycaster() const
{
  return m_state.m_raycaster;
}

void Pipeline::reset_forest()
{
  const size_t treeCount = 5;
  DecisionTree<SpaintVoxel::Label>::Settings dtSettings(m_state.m_resourcesDir + "/RaflSettings.xml");
  m_state.m_forest.reset(new RandomForest<SpaintVoxel::Label>(treeCount, dtSettings));
}

bool Pipeline::run_main_section()
{
  if(!m_state.m_imageSourceEngine->hasMoreImages()) return false;
  m_slamSection.run(m_state);
  return true;
}

void Pipeline::run_mode_specific_section(const RenderState_CPtr& renderState)
{
  switch(m_state.m_mode)
  {
    case PIPELINEMODE_FEATURE_INSPECTION:
      m_featureInspectionSection.run(m_state, renderState);
      break;
    case PIPELINEMODE_PREDICTION:
      m_predictionSection.run(m_state, renderState);
      break;
    case PIPELINEMODE_PROPAGATION:
      m_propagationSection.run(m_state, renderState);
      break;
    case PIPELINEMODE_SMOOTHING:
      m_smoothingSection.run(m_state, renderState);
      break;
    case PIPELINEMODE_TRAIN_AND_PREDICT:
    {
      static bool trainThisFrame = false;
      trainThisFrame = !trainThisFrame;

      if(trainThisFrame) m_trainingSection.run(m_state, renderState);
      else m_predictionSection.run(m_state, renderState);;

      break;
    }
    case PIPELINEMODE_TRAINING:
      m_trainingSection.run(m_state, renderState);
      break;
    default:
      break;
  }
}

void Pipeline::set_fusion_enabled(bool fusionEnabled)
{
  m_state.m_fusionEnabled = fusionEnabled;
}

void Pipeline::set_mode(PipelineMode mode)
{
#ifdef WITH_OPENCV
  // If we are switching out of feature inspection mode, destroy the feature inspection window.
  if(m_state.m_mode == PIPELINEMODE_FEATURE_INSPECTION && mode != PIPELINEMODE_FEATURE_INSPECTION)
  {
    cv::destroyAllWindows();
  }
#endif

  m_state.m_mode = mode;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void Pipeline::initialise(const Settings_Ptr& settings)
{
  // Make sure that we're not trying to run on the GPU if CUDA support isn't enabled.
#ifndef WITH_CUDA
  if(settings->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
    std::cerr << "[spaint] CUDA support unavailable, reverting to the CPU implementation of InfiniTAM\n";
    settings->deviceType = ITMLibSettings::DEVICE_CPU;
  }
#endif

  // Determine the RGB and depth image sizes.
  Vector2i rgbImageSize = m_state.m_imageSourceEngine->getRGBImageSize();
  Vector2i depthImageSize = m_state.m_imageSourceEngine->getDepthImageSize();
  if(depthImageSize.x == -1 || depthImageSize.y == -1) depthImageSize = rgbImageSize;

  // Set up the RGB and raw depth images into which input is to be read each frame.
  m_state.m_inputRGBImage.reset(new ITMUChar4Image(rgbImageSize, true, true));
  m_state.m_inputRawDepthImage.reset(new ITMShortImage(depthImageSize, true, true));

  // Set up the scene.
  MemoryDeviceType memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  Model::Scene_Ptr scene(new Model::Scene(&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED, memoryType));

  // Set up the InfiniTAM engines and view builder.
  m_state.m_lowLevelEngine.reset(ITMLowLevelEngineFactory::MakeLowLevelEngine(settings->deviceType));
  m_state.m_viewBuilder.reset(ITMViewBuilderFactory::MakeViewBuilder(&m_state.m_imageSourceEngine->getCalib(), settings->deviceType));
  VisualisationEngine_Ptr visualisationEngine(ITMVisualisationEngineFactory::MakeVisualisationEngine<SpaintVoxel,ITMVoxelIndex>(settings->deviceType));

  // Set up the dense mapper and tracking controller.
  m_state.m_denseMapper.reset(new ITMDenseMapper<SpaintVoxel,ITMVoxelIndex>(settings.get()));
  m_state.m_denseMapper->ResetScene(scene.get());
  setup_tracker(settings, scene, rgbImageSize, depthImageSize);
  m_state.m_trackingController.reset(new ITMTrackingController(m_state.m_tracker.get(), settings.get()));

  // Set up the live render state.
  Vector2i trackedImageSize = m_state.m_trackingController->GetTrackedImageSize(rgbImageSize, depthImageSize);
  RenderState_Ptr liveRenderState(visualisationEngine->CreateRenderState(scene.get(), trackedImageSize));

  // Set up the spaint model, raycaster and interactor.
  TrackingState_Ptr trackingState(new ITMTrackingState(trackedImageSize, memoryType));
  m_state.m_tracker->UpdateInitialPose(trackingState.get());
  m_state.m_model.reset(new Model(scene, rgbImageSize, depthImageSize, trackingState, settings, m_state.m_resourcesDir));
  m_state.m_raycaster.reset(new Raycaster(m_state.m_model, visualisationEngine, liveRenderState));
  m_state.m_interactor.reset(new Interactor(m_state.m_model));

  // Set up the label propagator.
  const int raycastResultSize = depthImageSize.width * depthImageSize.height;
  m_state.m_labelPropagator = LabelPropagatorFactory::make_label_propagator(raycastResultSize, settings->deviceType);

  // Set up the label smoother.
  const size_t maxLabelCount = m_state.m_model->get_label_manager()->get_max_label_count();
  m_state.m_labelSmoother = LabelSmootherFactory::make_label_smoother(maxLabelCount, settings->deviceType);

  // Set the maximum numbers of voxels to use for prediction and training.
  // FIXME: These values shouldn't be hard-coded here ultimately.
#ifndef USE_LOW_POWER_MODE
  m_state.m_maxPredictionVoxelCount = 8192;
#else
  m_state.m_maxPredictionVoxelCount = 512;
#endif
  m_state.m_maxTrainingVoxelsPerLabel = 128;
  const size_t maxTrainingVoxelCount = maxLabelCount * m_state.m_maxTrainingVoxelsPerLabel;

  // Set up the voxel samplers.
  const unsigned int seed = 12345;
  m_state.m_predictionSampler = VoxelSamplerFactory::make_uniform_sampler(raycastResultSize, seed, settings->deviceType);
  m_state.m_trainingSampler = VoxelSamplerFactory::make_per_label_sampler(maxLabelCount, m_state.m_maxTrainingVoxelsPerLabel, raycastResultSize, seed, settings->deviceType);

  // Set up the feature calculator.
  // FIXME: These values shouldn't be hard-coded here ultimately.
  m_state.m_patchSize = 13;
  const float patchSpacing = 0.01f / settings->sceneParams.voxelSize; // 10mm = 0.01m (dividing by the voxel size, which is in m, expresses the spacing in voxels)
  const size_t binCount = 36;                                         // 10 degrees per bin

  m_state.m_featureCalculator = FeatureCalculatorFactory::make_vop_feature_calculator(
    std::max(m_state.m_maxPredictionVoxelCount, maxTrainingVoxelCount),
    m_state.m_patchSize, patchSpacing, binCount, settings->deviceType
  );

  // Set up the memory blocks needed for prediction and training.
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
  const size_t featureCount = m_state.m_featureCalculator->get_feature_count();
  m_state.m_predictionFeaturesMB = mbf.make_block<float>(m_state.m_maxPredictionVoxelCount * featureCount);
  m_state.m_predictionLabelsMB = mbf.make_block<SpaintVoxel::PackedLabel>(m_state.m_maxPredictionVoxelCount);
  m_state.m_predictionVoxelLocationsMB = mbf.make_block<Vector3s>(m_state.m_maxPredictionVoxelCount);
  m_state.m_trainingFeaturesMB = mbf.make_block<float>(maxTrainingVoxelCount * featureCount);
  m_state.m_trainingLabelMaskMB = mbf.make_block<bool>(maxLabelCount);
  m_state.m_trainingVoxelCountsMB = mbf.make_block<unsigned int>(maxLabelCount);
  m_state.m_trainingVoxelLocationsMB = mbf.make_block<Vector3s>(maxTrainingVoxelCount);

  // Register the relevant decision function generators with the factory.
  DecisionFunctionGeneratorFactory<SpaintVoxel::Label>::instance().register_maker(
    SpaintDecisionFunctionGenerator::get_static_type(),
    &SpaintDecisionFunctionGenerator::maker
  );

  // Set up the random forest.
  reset_forest();

  // Set up the pose database and the relocaliser.
  m_state.m_poseDatabase.reset(new PoseDatabase);

  const float harvestingThreshold = 0.2f;
  const int numFerns = 500;
  const int numDecisionsPerFern = 4;
  m_state.m_relocaliser.reset(new Relocaliser(
    depthImageSize,
    Vector2f(settings->sceneParams.viewFrustum_min, settings->sceneParams.viewFrustum_max),
    harvestingThreshold, numFerns, numDecisionsPerFern
  ));

  m_state.m_fusionEnabled = true;
  m_state.m_mode = PIPELINEMODE_NORMAL;
}

ITMTracker *Pipeline::make_hybrid_tracker(ITMTracker *primaryTracker, const Settings_Ptr& settings, const Model::Scene_Ptr& scene,
                                          const Vector2i& rgbImageSize, const Vector2i& depthImageSize) const
{
  ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
  compositeTracker->SetTracker(primaryTracker, 0);
  compositeTracker->SetTracker(
    ITMTrackerFactory<SpaintVoxel,ITMVoxelIndex>::Instance().MakeICPTracker(
      rgbImageSize, depthImageSize, settings->deviceType, ORUtils::KeyValueConfig(settings->trackerConfig), m_state.m_lowLevelEngine.get(), m_state.m_imuCalibrator.get(), scene.get()
    ), 1
  );
  return compositeTracker;
}

void Pipeline::setup_tracker(const Settings_Ptr& settings, const Model::Scene_Ptr& scene, const Vector2i& rgbImageSize, const Vector2i& depthImageSize)
{
  m_state.m_fallibleTracker = NULL;

  switch(m_state.m_trackerType)
  {
    case TRACKER_RIFT:
    {
#ifdef WITH_OVR
      m_state.m_tracker.reset(make_hybrid_tracker(new RiftTracker, settings, scene, rgbImageSize, depthImageSize));
      break;
#else
      // This should never happen as things stand - we never try to use the Rift tracker if Rift support isn't available.
      throw std::runtime_error("Error: Rift support not currently available. Reconfigure in CMake with the WITH_OVR option set to on.");
#endif
    }
    case TRACKER_ROBUSTVICON:
    {
#ifdef WITH_VICON
      m_fallibleTracker = new RobustViconTracker(m_trackerParams, "kinect", rgbImageSize, depthImageSize, settings, m_lowLevelEngine, scene);
      m_tracker.reset(m_fallibleTracker);
      break;
#else
      // This should never happen as things stand - we never try to use the robust Vicon tracker if Vicon support isn't available.
      throw std::runtime_error("Error: Vicon support not currently available. Reconfigure in CMake with the WITH_VICON option set to on.");
#endif
    }
    case TRACKER_VICON:
    {
#ifdef WITH_VICON
      m_fallibleTracker = new ViconTracker(m_trackerParams, "kinect");
      m_tracker.reset(make_hybrid_tracker(m_fallibleTracker, settings, scene, rgbImageSize, depthImageSize));
      break;
#else
      // This should never happen as things stand - we never try to use the Vicon tracker if Vicon support isn't available.
      throw std::runtime_error("Error: Vicon support not currently available. Reconfigure in CMake with the WITH_VICON option set to on.");
#endif
    }
    default: // TRACKER_INFINITAM
    {
      m_state.m_imuCalibrator.reset(new ITMIMUCalibrator_iPad);
      m_state.m_tracker.reset(ITMTrackerFactory<SpaintVoxel,ITMVoxelIndex>::Instance().Make(
        rgbImageSize, depthImageSize, settings.get(), m_state.m_lowLevelEngine.get(), m_state.m_imuCalibrator.get(), scene.get()
      ));
    }
  }
}
