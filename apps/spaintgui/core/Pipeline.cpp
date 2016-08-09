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

  const Raycaster::RenderState_Ptr& liveRenderState = m_state.m_raycaster->get_live_render_state();
  const Model::Scene_Ptr& scene = m_state.m_model->get_scene();
  const Model::TrackingState_Ptr& trackingState = m_state.m_model->get_tracking_state();
  const Model::View_Ptr& view = m_state.m_model->get_view();

  // Get the next frame.
  ITMView *newView = view.get();
  m_state.m_imageSourceEngine->getImages(m_state.m_inputRGBImage.get(), m_state.m_inputRawDepthImage.get());
  const bool useBilateralFilter = false;
  m_state.m_viewBuilder->UpdateView(&newView, m_state.m_inputRGBImage.get(), m_state.m_inputRawDepthImage.get(), useBilateralFilter);
  m_state.m_model->set_view(newView);

  // Track the camera (we can only do this once we've started reconstructing the model because we need something to track against).
  SE3Pose oldPose(*trackingState->pose_d);
  if(m_state.m_fusedFramesCount > 0) m_state.m_trackingController->Track(trackingState.get(), view.get());

  // Determine the tracking quality, taking into account the failure mode being used.
  ITMTrackingState::TrackingResult trackerResult = trackingState->trackerResult;
  switch(m_state.m_model->get_settings()->behaviourOnFailure)
  {
    case ITMLibSettings::FAILUREMODE_RELOCALISE:
    {
      // Copy the current depth input across to the CPU for use by the relocaliser.
      view->depth->UpdateHostFromDevice();

      // Decide whether or not the relocaliser should consider using this frame as a keyframe.
      bool considerKeyframe = false;
      if(trackerResult == ITMTrackingState::TRACKING_GOOD)
      {
        if(m_state.m_keyframeDelay == 0) considerKeyframe = true;
        else --m_state.m_keyframeDelay;
      }

      // Process the current depth image using the relocaliser. This attempts to find the nearest keyframe (if any)
      // that is currently in the database, and may add the current frame as a new keyframe if the tracking has been
      // good for some time and the current frame differs sufficiently from the existing keyframes.
      int nearestNeighbour;
      int keyframeID = m_state.m_relocaliser->ProcessFrame(view->depth, 1, &nearestNeighbour, NULL, considerKeyframe);

      if(keyframeID >= 0)
      {
        // If the relocaliser added the current frame as a new keyframe, store its pose in the pose database.
        // Note that a new keyframe will only have been added if the tracking quality for this frame was good.
        m_state.m_poseDatabase->storePose(keyframeID, *trackingState->pose_d, 0);
      }
      else if(trackerResult == ITMTrackingState::TRACKING_FAILED && nearestNeighbour != -1)
      {
        // If the tracking failed but a nearest keyframe was found by the relocaliser, reset the pose to that
        // of the keyframe and rerun the tracker for this frame.
        trackingState->pose_d->SetFrom(&m_state.m_poseDatabase->retrievePose(nearestNeighbour).pose);

        const bool resetVisibleList = true;
        m_state.m_denseMapper->UpdateVisibleList(view.get(), trackingState.get(), scene.get(), liveRenderState.get(), resetVisibleList);
        m_state.m_trackingController->Prepare(trackingState.get(), scene.get(), view.get(), m_state.m_raycaster->get_visualisation_engine().get(), liveRenderState.get());
        m_state.m_trackingController->Track(trackingState.get(), view.get());
        trackerResult = trackingState->trackerResult;

        // Set the number of frames for which the tracking quality must be good before the relocaliser can consider
        // adding a new keyframe.
        m_state.m_keyframeDelay = 10;
      }

      break;
    }
    case ITMLibSettings::FAILUREMODE_STOP_INTEGRATION:
    {
      // Since we're not using relocalisation, treat tracking failures like poor tracking,
      // on the basis that it's better to try to keep going than to fail completely.
      if(trackerResult == ITMTrackingState::TRACKING_FAILED) trackerResult = ITMTrackingState::TRACKING_POOR;

      break;
    }
    case ITMLibSettings::FAILUREMODE_IGNORE:
    default:
    {
      // If we're completely ignoring poor or failed tracking, treat the tracking quality as good.
      trackerResult = ITMTrackingState::TRACKING_GOOD;
      break;
    }
  }

  // Decide whether or not fusion should be run.
  bool runFusion = m_state.m_fusionEnabled;
  if(trackerResult == ITMTrackingState::TRACKING_FAILED ||
     (trackerResult == ITMTrackingState::TRACKING_POOR && m_state.m_fusedFramesCount >= m_state.m_initialFramesToFuse) ||
     (m_state.m_fallibleTracker && m_state.m_fallibleTracker->lost_tracking()))
  {
    runFusion = false;
  }

  if(runFusion)
  {
    // Run the fusion process.
    m_state.m_denseMapper->ProcessFrame(view.get(), trackingState.get(), scene.get(), liveRenderState.get());
    ++m_state.m_fusedFramesCount;
  }
  else if(trackerResult != ITMTrackingState::TRACKING_FAILED)
  {
    // If we're not fusing, but the tracking has not completely failed, update the list of visible blocks so that things are kept up to date.
    m_state.m_denseMapper->UpdateVisibleList(view.get(), trackingState.get(), scene.get(), liveRenderState.get());
  }
  else
  {
    // If the tracking has completely failed, restore the pose from the previous frame.
    *trackingState->pose_d = oldPose;
  }

  // Raycast from the live camera position to prepare for tracking in the next frame.
  m_state.m_trackingController->Prepare(trackingState.get(), scene.get(), view.get(), m_state.m_raycaster->get_visualisation_engine().get(), liveRenderState.get());

  // If the current sub-engine has run out of images, disable fusion.
  if(!m_state.m_imageSourceEngine->getCurrentSubengine()->hasMoreImages()) m_state.m_fusionEnabled = false;

  return true;
}

void Pipeline::run_mode_specific_section(const RenderState_CPtr& renderState)
{
  switch(m_state.m_mode)
  {
    case PIPELINEMODE_FEATURE_INSPECTION:
      run_feature_inspection_section(renderState);
      break;
    case PIPELINEMODE_PREDICTION:
      run_prediction_section(renderState);
      break;
    case PIPELINEMODE_PROPAGATION:
      run_propagation_section(renderState);
      break;
    case PIPELINEMODE_SMOOTHING:
      run_smoothing_section(renderState);
      break;
    case PIPELINEMODE_TRAIN_AND_PREDICT:
    {
      static bool trainThisFrame = false;
      trainThisFrame = !trainThisFrame;

      if(trainThisFrame) run_training_section(renderState);
      else run_prediction_section(renderState);

      break;
    }
    case PIPELINEMODE_TRAINING:
      run_training_section(renderState);
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

  m_state.m_featureInspectionWindowName = "Feature Inspection";
  m_state.m_fusedFramesCount = 0;
  m_state.m_fusionEnabled = true;
  m_state.m_keyframeDelay = 0;
  m_state.m_mode = PIPELINEMODE_NORMAL;

  // FIXME: This value should be passed in rather than hard-coded.
  m_state.m_initialFramesToFuse = 50;
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

void Pipeline::run_feature_inspection_section(const RenderState_CPtr& renderState)
{
  // Get the voxels (if any) selected by the user (prior to selection transformation).
  Selector::Selection_CPtr selection = m_state.m_interactor->get_selector()->get_selection();

  // If the user hasn't selected a single voxel, early out.
  if(!selection || selection->dataSize != 1) return;

  // Calculate the feature descriptor for the selected voxel.
  boost::shared_ptr<ORUtils::MemoryBlock<float> > featuresMB = MemoryBlockFactory::instance().make_block<float>(m_state.m_featureCalculator->get_feature_count());
  m_state.m_featureCalculator->calculate_features(*selection, m_state.m_model->get_scene().get(), *featuresMB);

#ifdef WITH_OPENCV
  // Convert the feature descriptor into an OpenCV image and show it in a window.
  featuresMB->UpdateHostFromDevice();
  const float *features = featuresMB->GetData(MEMORYDEVICE_CPU);
  const int patchSize = static_cast<int>(m_state.m_patchSize);
  cv::Mat3b featureInspectionImage = OpenCVUtil::make_rgb_image(features, patchSize, patchSize);

  const float scaleFactor = 10.0f;
  cv::resize(featureInspectionImage, featureInspectionImage, cv::Size(), scaleFactor, scaleFactor, CV_INTER_NN);

  cv::imshow(m_state.m_featureInspectionWindowName, featureInspectionImage);
  const int delayMs = 1;
  cv::waitKey(delayMs);  // this is required in order to make OpenCV actually show the window
#endif
}

void Pipeline::run_prediction_section(const RenderState_CPtr& samplingRenderState)
{
  // If we haven't been provided with a camera position from which to sample, early out.
  if(!samplingRenderState) return;

  // If the random forest is not yet valid, early out.
  if(!m_state.m_forest->is_valid()) return;

  // Sample some voxels for which to predict labels.
  m_state.m_predictionSampler->sample_voxels(samplingRenderState->raycastResult, m_state.m_maxPredictionVoxelCount, *m_state.m_predictionVoxelLocationsMB);

  // Calculate feature descriptors for the sampled voxels.
  m_state.m_featureCalculator->calculate_features(*m_state.m_predictionVoxelLocationsMB, m_state.m_model->get_scene().get(), *m_state.m_predictionFeaturesMB);
  std::vector<Descriptor_CPtr> descriptors = ForestUtil::make_descriptors(*m_state.m_predictionFeaturesMB, m_state.m_maxPredictionVoxelCount, m_state.m_featureCalculator->get_feature_count());

  // Predict labels for the voxels based on the feature descriptors.
  SpaintVoxel::PackedLabel *labels = m_state.m_predictionLabelsMB->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < static_cast<int>(m_state.m_maxPredictionVoxelCount); ++i)
  {
    labels[i] = SpaintVoxel::PackedLabel(m_state.m_forest->predict(descriptors[i]), SpaintVoxel::LG_FOREST);
  }

  m_state.m_predictionLabelsMB->UpdateDeviceFromHost();

  // Mark the voxels with their predicted labels.
  m_state.m_interactor->mark_voxels(m_state.m_predictionVoxelLocationsMB, m_state.m_predictionLabelsMB);
}

void Pipeline::run_propagation_section(const RenderState_CPtr& renderState)
{
  m_state.m_labelPropagator->propagate_label(m_state.m_interactor->get_semantic_label(), renderState->raycastResult, m_state.m_model->get_scene().get());
}

void Pipeline::run_smoothing_section(const RenderState_CPtr& renderState)
{
  m_state.m_labelSmoother->smooth_labels(renderState->raycastResult, m_state.m_model->get_scene().get());
}

void Pipeline::run_training_section(const RenderState_CPtr& samplingRenderState)
{
  // If we haven't been provided with a camera position from which to sample, early out.
  if(!samplingRenderState) return;

  // Calculate a mask indicating the labels that are currently in use and from which we want to train.
  // Note that we deliberately avoid training from the background label (0), since the entire scene is
  // initially labelled as background and so training from the background would cause us to learn
  // incorrect labels for non-background things.
  LabelManager_CPtr labelManager = m_state.m_model->get_label_manager();
  const size_t maxLabelCount = labelManager->get_max_label_count();
  bool *labelMask = m_state.m_trainingLabelMaskMB->GetData(MEMORYDEVICE_CPU);
  labelMask[0] = false;
  for(size_t i = 1; i < maxLabelCount; ++i)
  {
    labelMask[i] = labelManager->has_label(static_cast<SpaintVoxel::Label>(i));
  }
  m_state.m_trainingLabelMaskMB->UpdateDeviceFromHost();

  // Sample voxels from the scene to use for training the random forest.
  const ORUtils::Image<Vector4f> *raycastResult = samplingRenderState->raycastResult;
  m_state.m_trainingSampler->sample_voxels(raycastResult, m_state.m_model->get_scene().get(), *m_state.m_trainingLabelMaskMB, *m_state.m_trainingVoxelLocationsMB, *m_state.m_trainingVoxelCountsMB);

#if DEBUGGING
  // Output the numbers of voxels sampled for each label (for debugging purposes).
  for(size_t i = 0; i < m_state.m_trainingVoxelCountsMB->dataSize; ++i)
  {
    std::cout << m_state.m_trainingVoxelCountsMB->GetData(MEMORYDEVICE_CPU)[i] << ' ';
  }
  std::cout << '\n';

  // Make sure that the sampled voxels are available on the CPU so that they can be checked.
  m_state.m_trainingVoxelLocationsMB->UpdateHostFromDevice();
#endif

  // Compute feature vectors for the sampled voxels.
  m_state.m_featureCalculator->calculate_features(*m_state.m_trainingVoxelLocationsMB, m_state.m_model->get_scene().get(), *m_state.m_trainingFeaturesMB);

  // Make the training examples.
  typedef boost::shared_ptr<const Example<SpaintVoxel::Label> > Example_CPtr;
  std::vector<Example_CPtr> examples = ForestUtil::make_examples<SpaintVoxel::Label>(
    *m_state.m_trainingFeaturesMB,
    *m_state.m_trainingVoxelCountsMB,
    m_state.m_featureCalculator->get_feature_count(),
    m_state.m_maxTrainingVoxelsPerLabel,
    maxLabelCount
  );

  // Train the forest.
  const size_t splitBudget = 20;
  m_state.m_forest->add_examples(examples);
  m_state.m_forest->train(splitBudget);
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
