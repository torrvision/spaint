/**
 * spaint: SpaintPipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "core/SpaintPipeline.h"
using namespace rafl;

#ifdef WITH_OPENNI
#include <Engine/OpenNIEngine.h>
#endif
#include <ITMLib/Engine/ITMRenTracker.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.cpp>
using namespace InfiniTAM::Engine;

#include "features/FeatureCalculatorFactory.h"
#include "propagation/LabelPropagatorFactory.h"
#include "randomforest/ForestUtil.h"
#include "randomforest/SpaintDecisionFunctionGenerator.h"
#include "sampling/VoxelSamplerFactory.h"
#include "trackers/RefineWithICPTracker.h"
#include "util/MemoryBlockFactory.h"

#ifdef WITH_OPENCV
#include "ocv/OpenCVUtil.h"
#endif

#ifdef WITH_OVR
#include "trackers/RiftTracker.h"
#endif

#define DEBUGGING 1

namespace spaint {

//#################### CONSTRUCTORS ####################

#ifdef WITH_OPENNI
SpaintPipeline::SpaintPipeline(const std::string& calibrationFilename, const boost::optional<std::string>& openNIDeviceURI, const Settings_Ptr& settings,
                               const std::string& resourcesDir, TrackerType trackerType, const std::string& trackerParams, bool useInternalCalibration)
: m_resourcesDir(resourcesDir), m_trackerParams(trackerParams), m_trackerType(trackerType)
{
  m_imageSourceEngine.reset(new OpenNIEngine(calibrationFilename.c_str(), openNIDeviceURI ? openNIDeviceURI->c_str() : NULL, useInternalCalibration));
  initialise(settings);
}
#endif

SpaintPipeline::SpaintPipeline(const std::string& calibrationFilename, const std::string& rgbImageMask, const std::string& depthImageMask,
                               const Settings_Ptr& settings, const std::string& resourcesDir)
: m_resourcesDir(resourcesDir)
{
  m_imageSourceEngine.reset(new ImageFileReader(calibrationFilename.c_str(), rgbImageMask.c_str(), depthImageMask.c_str()));
  initialise(settings);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool SpaintPipeline::get_fusion_enabled() const
{
  return m_fusionEnabled;
}

const SpaintInteractor_Ptr& SpaintPipeline::get_interactor()
{
  return m_interactor;
}

SpaintPipeline::Mode SpaintPipeline::get_mode() const
{
  return m_mode;
}

const SpaintModel_Ptr& SpaintPipeline::get_model()
{
  return m_model;
}

SpaintModel_CPtr SpaintPipeline::get_model() const
{
  return m_model;
}

const SpaintRaycaster_Ptr& SpaintPipeline::get_raycaster()
{
  return m_raycaster;
}

SpaintRaycaster_CPtr SpaintPipeline::get_raycaster() const
{
  return m_raycaster;
}

void SpaintPipeline::reset_forest()
{
  const size_t treeCount = 5;
  DecisionTree<SpaintVoxel::Label>::Settings dtSettings(m_resourcesDir + "/RaflSettings.xml");
  m_forest.reset(new RandomForest<SpaintVoxel::Label>(treeCount, dtSettings));
}

void SpaintPipeline::run_main_section()
{
  if(!m_imageSourceEngine->hasMoreImages()) return;

  const SpaintRaycaster::RenderState_Ptr& liveRenderState = m_raycaster->get_live_render_state();
  const SpaintModel::Scene_Ptr& scene = m_model->get_scene();
  const SpaintModel::TrackingState_Ptr& trackingState = m_model->get_tracking_state();
  const SpaintModel::View_Ptr& view = m_model->get_view();

  // Get the next frame.
  ITMView *newView = view.get();
  m_imageSourceEngine->getImages(m_inputRGBImage.get(), m_inputRawDepthImage.get());
  const bool useBilateralFilter = false;
  m_viewBuilder->UpdateView(&newView, m_inputRGBImage.get(), m_inputRawDepthImage.get(), useBilateralFilter);
  m_model->set_view(newView);

  // Track the camera (we can only do this once we've started reconstructing the model because we need something to track against).
  if(m_reconstructionStarted) m_trackingController->Track(trackingState.get(), view.get());

  // Determine whether or not fusion should be run.
  bool runFusion = m_fusionEnabled;
#ifdef WITH_VICON
  if(m_trackerType == TRACKER_VICON && m_viconTracker->lost_tracking()) runFusion = false;

  if(m_trackerType == TRACKER_VICON && dynamic_cast<const RefineWithICPTracker*>(m_tracker.get())->lost_tracking()) runFusion = false;
  std::cout << "Run Fusion: " << runFusion << '\n';
#endif

  if(runFusion)
  {
    // Run the fusion process.
    m_denseMapper->ProcessFrame(view.get(), trackingState.get(), scene.get(), liveRenderState.get());
    m_reconstructionStarted = true;
  }
  else
  {
    // Update the list of visible blocks so that things are kept up to date even when we're not fusing.
    m_denseMapper->UpdateVisibleList(view.get(), trackingState.get(), scene.get(), liveRenderState.get());
  }

  // Raycast from the live camera position to prepare for tracking in the next frame.
  m_trackingController->Prepare(trackingState.get(), view.get(), liveRenderState.get());
}

void SpaintPipeline::run_mode_specific_section(const RenderState_CPtr& renderState)
{
  switch(m_mode)
  {
    case MODE_FEATURE_INSPECTION:
      run_feature_inspection_section(renderState);
      break;
    case MODE_PREDICTION:
      run_prediction_section(renderState);
      break;
    case MODE_PROPAGATION:
      run_propagation_section(renderState);
      break;
    case MODE_TRAIN_AND_PREDICT:
    {
      static bool trainThisFrame = false;
      trainThisFrame = !trainThisFrame;

      if(trainThisFrame) run_training_section(renderState);
      else run_prediction_section(renderState);

      break;
    }
    case MODE_TRAINING:
      run_training_section(renderState);
      break;
    default:
      break;
  }
}

void SpaintPipeline::set_fusion_enabled(bool fusionEnabled)
{
  m_fusionEnabled = fusionEnabled;
}

void SpaintPipeline::set_mode(Mode mode)
{
#ifdef WITH_OPENCV
  // If we are switching out of feature inspection mode, destroy the feature inspection window.
  if(m_mode == MODE_FEATURE_INSPECTION && mode != MODE_FEATURE_INSPECTION)
  {
    cv::destroyAllWindows();
  }
#endif

  m_mode = mode;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void SpaintPipeline::initialise(const Settings_Ptr& settings)
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
  Vector2i rgbImageSize = m_imageSourceEngine->getRGBImageSize();
  Vector2i depthImageSize = m_imageSourceEngine->getDepthImageSize();
  if(depthImageSize.x == -1 || depthImageSize.y == -1) depthImageSize = rgbImageSize;

  // Set up the RGB and raw depth images into which input is to be read each frame.
  m_inputRGBImage.reset(new ITMUChar4Image(rgbImageSize, true, true));
  m_inputRawDepthImage.reset(new ITMShortImage(depthImageSize, true, true));

  // Set up the scene.
  MemoryDeviceType memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  SpaintModel::Scene_Ptr scene(new SpaintModel::Scene(&settings->sceneParams, settings->useSwapping, memoryType));

  // Set up the InfiniTAM engines and view builder.
  const ITMRGBDCalib *calib = &m_imageSourceEngine->calib;
  VisualisationEngine_Ptr visualisationEngine;
  if(settings->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    // Use the CUDA implementations.
    m_lowLevelEngine.reset(new ITMLowLevelEngine_CUDA);
    m_viewBuilder.reset(new ITMViewBuilder_CUDA(calib));
    visualisationEngine.reset(new ITMVisualisationEngine_CUDA<SpaintVoxel,ITMVoxelIndex>(scene.get()));
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU to false if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    // Use the CPU implementations.
    m_lowLevelEngine.reset(new ITMLowLevelEngine_CPU);
    m_viewBuilder.reset(new ITMViewBuilder_CPU(calib));
    visualisationEngine.reset(new ITMVisualisationEngine_CPU<SpaintVoxel,ITMVoxelIndex>(scene.get()));
  }

  // Set up the live render state.
  Vector2i trackedImageSize = ITMTrackingController::GetTrackedImageSize(settings.get(), rgbImageSize, depthImageSize);
  RenderState_Ptr liveRenderState(visualisationEngine->CreateRenderState(trackedImageSize));

  // Set up the dense mapper and tracking controller.
  m_denseMapper.reset(new ITMDenseMapper<SpaintVoxel,ITMVoxelIndex>(settings.get()));
  setup_tracker(settings, scene, trackedImageSize);
  m_trackingController.reset(new ITMTrackingController(m_tracker.get(), visualisationEngine.get(), m_lowLevelEngine.get(), settings.get()));

  // Set up the spaint model, raycaster and interactor.
  TrackingState_Ptr trackingState(m_trackingController->BuildTrackingState(trackedImageSize));
  m_tracker->UpdateInitialPose(trackingState.get());
  m_model.reset(new SpaintModel(scene, rgbImageSize, depthImageSize, trackingState, settings, m_resourcesDir));
  m_raycaster.reset(new SpaintRaycaster(m_model, visualisationEngine, liveRenderState));
  m_interactor.reset(new SpaintInteractor(m_model));

  // Set up the label propagator.
  const int raycastResultSize = depthImageSize.width * depthImageSize.height;
  m_labelPropagator = LabelPropagatorFactory::make_label_propagator(raycastResultSize, settings->deviceType);

  // Set the maximum numbers of voxels to use for prediction and training.
  // FIXME: These values shouldn't be hard-coded here ultimately.
  const size_t maxLabelCount = m_model->get_label_manager()->get_max_label_count();
#ifndef USE_LOW_POWER_MODE
  m_maxPredictionVoxelCount = 8192;
#else
  m_maxPredictionVoxelCount = 512;
#endif
  m_maxTrainingVoxelsPerLabel = 128;
  const size_t maxTrainingVoxelCount = maxLabelCount * m_maxTrainingVoxelsPerLabel;

  // Set up the voxel samplers.
  const unsigned int seed = 12345;
  m_predictionSampler = VoxelSamplerFactory::make_uniform_sampler(raycastResultSize, seed, settings->deviceType);
  m_trainingSampler = VoxelSamplerFactory::make_per_label_sampler(maxLabelCount, m_maxTrainingVoxelsPerLabel, raycastResultSize, seed, settings->deviceType);

  // Set up the feature calculator.
  // FIXME: These values shouldn't be hard-coded here ultimately.
  m_patchSize = 13;
  const float patchSpacing = 0.01f / settings->sceneParams.voxelSize; // 10mm = 0.01m (dividing by the voxel size, which is in m, expresses the spacing in voxels)
  const size_t binCount = 36;                                         // 10 degrees per bin

  m_featureCalculator = FeatureCalculatorFactory::make_vop_feature_calculator(
    std::max(m_maxPredictionVoxelCount, maxTrainingVoxelCount),
    m_patchSize, patchSpacing, binCount, settings->deviceType
  );

  // Set up the memory blocks needed for prediction and training.
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
  const size_t featureCount = m_featureCalculator->get_feature_count();
  m_predictionFeaturesMB = mbf.make_block<float>(m_maxPredictionVoxelCount * featureCount);
  m_predictionLabelsMB = mbf.make_block<SpaintVoxel::PackedLabel>(m_maxPredictionVoxelCount);
  m_predictionVoxelLocationsMB = mbf.make_block<Vector3s>(m_maxPredictionVoxelCount);
  m_trainingFeaturesMB = mbf.make_block<float>(maxTrainingVoxelCount * featureCount);
  m_trainingLabelMaskMB = mbf.make_block<bool>(maxLabelCount);
  m_trainingVoxelCountsMB = mbf.make_block<unsigned int>(maxLabelCount);
  m_trainingVoxelLocationsMB = mbf.make_block<Vector3s>(maxTrainingVoxelCount);

  // Register the relevant decision function generators with the factory.
  DecisionFunctionGeneratorFactory<SpaintVoxel::Label>::instance().register_maker(
    SpaintDecisionFunctionGenerator::get_static_type(),
    &SpaintDecisionFunctionGenerator::maker
  );

  // Set up the random forest.
  reset_forest();

  m_featureInspectionWindowName = "Feature Inspection";
  m_fusionEnabled = true;
  m_mode = MODE_NORMAL;
  m_reconstructionStarted = false;
}

ITMTracker *SpaintPipeline::make_hybrid_tracker(ITMTracker *primaryTracker, const Settings_Ptr& settings, const SpaintModel::Scene_Ptr& scene, const Vector2i& trackedImageSize) const
{
  ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
  compositeTracker->SetTracker(primaryTracker, 0);
  compositeTracker->SetTracker(
    ITMTrackerFactory<SpaintVoxel,ITMVoxelIndex>::Instance().MakeICPTracker(
      trackedImageSize,
      settings.get(),
      m_lowLevelEngine.get(),
      m_imuCalibrator.get(),
      scene.get()
    ), 1
  );
  return compositeTracker;
}

void SpaintPipeline::run_feature_inspection_section(const RenderState_CPtr& renderState)
{
  // Get the voxels (if any) selected by the user (prior to selection transformation).
  Selector::Selection_CPtr selection = m_interactor->get_selector()->get_selection();

  // If the user hasn't selected a single voxel, early out.
  if(!selection || selection->dataSize != 1) return;

  // Calculate the feature descriptor for the selected voxel.
  boost::shared_ptr<ORUtils::MemoryBlock<float> > featuresMB = MemoryBlockFactory::instance().make_block<float>(m_featureCalculator->get_feature_count());
  m_featureCalculator->calculate_features(*selection, m_model->get_scene().get(), *featuresMB);

#ifdef WITH_OPENCV
  // Convert the feature descriptor into an OpenCV image and show it in a window.
  featuresMB->UpdateHostFromDevice();
  const float *features = featuresMB->GetData(MEMORYDEVICE_CPU);
  const int patchSize = static_cast<int>(m_patchSize);
  cv::Mat3b featureInspectionImage = OpenCVUtil::make_rgb_image(features, patchSize, patchSize);

  const float scaleFactor = 10.0f;
  cv::resize(featureInspectionImage, featureInspectionImage, cv::Size(), scaleFactor, scaleFactor, CV_INTER_NN);

  cv::imshow(m_featureInspectionWindowName, featureInspectionImage);
  const int delayMs = 1;
  cv::waitKey(delayMs);  // this is required in order to make OpenCV actually show the window
#endif
}

void SpaintPipeline::run_prediction_section(const RenderState_CPtr& samplingRenderState)
{
  // If we haven't been provided with a camera position from which to sample, early out.
  if(!samplingRenderState) return;

  // If the random forest is not yet valid, early out.
  if(!m_forest->is_valid()) return;

  // Sample some voxels for which to predict labels.
  m_predictionSampler->sample_voxels(samplingRenderState->raycastResult, m_maxPredictionVoxelCount, *m_predictionVoxelLocationsMB);

  // Calculate feature descriptors for the sampled voxels.
  m_featureCalculator->calculate_features(*m_predictionVoxelLocationsMB, m_model->get_scene().get(), *m_predictionFeaturesMB);
  std::vector<Descriptor_CPtr> descriptors = ForestUtil::make_descriptors(*m_predictionFeaturesMB, m_maxPredictionVoxelCount, m_featureCalculator->get_feature_count());

  // Predict labels for the voxels based on the feature descriptors.
  SpaintVoxel::PackedLabel *labels = m_predictionLabelsMB->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < static_cast<int>(m_maxPredictionVoxelCount); ++i)
  {
    labels[i] = SpaintVoxel::PackedLabel(m_forest->predict(descriptors[i]), SpaintVoxel::LG_FOREST);
  }

  m_predictionLabelsMB->UpdateDeviceFromHost();

  // Mark the voxels with their predicted labels.
  m_interactor->mark_voxels(m_predictionVoxelLocationsMB, m_predictionLabelsMB);
}

void SpaintPipeline::run_propagation_section(const RenderState_CPtr& renderState)
{
  m_labelPropagator->propagate_label(m_interactor->get_semantic_label(), renderState->raycastResult, m_model->get_scene().get());
}

void SpaintPipeline::run_training_section(const RenderState_CPtr& samplingRenderState)
{
  // If we haven't been provided with a camera position from which to sample, early out.
  if(!samplingRenderState) return;

  // Calculate a mask indicating the labels that are currently in use and from which we want to train.
  // Note that we deliberately avoid training from the background label (0), since the entire scene is
  // initially labelled as background and so training from the background would cause us to learn
  // incorrect labels for non-background things.
  LabelManager_CPtr labelManager = m_model->get_label_manager();
  const size_t maxLabelCount = labelManager->get_max_label_count();
  bool *labelMask = m_trainingLabelMaskMB->GetData(MEMORYDEVICE_CPU);
  labelMask[0] = false;
  for(size_t i = 1; i < maxLabelCount; ++i)
  {
    labelMask[i] = labelManager->has_label(static_cast<SpaintVoxel::Label>(i));
  }
  m_trainingLabelMaskMB->UpdateDeviceFromHost();

  // Sample voxels from the scene to use for training the random forest.
  const ORUtils::Image<Vector4f> *raycastResult = samplingRenderState->raycastResult;
  m_trainingSampler->sample_voxels(raycastResult, m_model->get_scene().get(), *m_trainingLabelMaskMB, *m_trainingVoxelLocationsMB, *m_trainingVoxelCountsMB);

#if DEBUGGING
  // Output the numbers of voxels sampled for each label (for debugging purposes).
  for(size_t i = 0; i < m_trainingVoxelCountsMB->dataSize; ++i)
  {
    std::cout << m_trainingVoxelCountsMB->GetData(MEMORYDEVICE_CPU)[i] << ' ';
  }
  std::cout << '\n';

  // Make sure that the sampled voxels are available on the CPU so that they can be checked.
  m_trainingVoxelLocationsMB->UpdateHostFromDevice();
#endif

  // Compute feature vectors for the sampled voxels.
  m_featureCalculator->calculate_features(*m_trainingVoxelLocationsMB, m_model->get_scene().get(), *m_trainingFeaturesMB);

  // Make the training examples.
  typedef boost::shared_ptr<const Example<SpaintVoxel::Label> > Example_CPtr;
  std::vector<Example_CPtr> examples = ForestUtil::make_examples<SpaintVoxel::Label>(
    *m_trainingFeaturesMB,
    *m_trainingVoxelCountsMB,
    m_featureCalculator->get_feature_count(),
    m_maxTrainingVoxelsPerLabel,
    maxLabelCount
  );

  // Train the forest.
  const size_t splitBudget = 128;
  m_forest->add_examples(examples);
  m_forest->train(splitBudget);
}

void SpaintPipeline::setup_tracker(const Settings_Ptr& settings, const SpaintModel::Scene_Ptr& scene, const Vector2i& trackedImageSize)
{
  switch(m_trackerType)
  {
    case TRACKER_RIFT:
    {
#ifdef WITH_OVR
      m_tracker.reset(make_hybrid_tracker(new RiftTracker, settings, scene, trackedImageSize));
      break;
#else
      // This should never happen as things stand - we never try to use the Rift tracker if Rift support isn't available.
      throw std::runtime_error("Error: Rift support not currently available. Reconfigure in CMake with the WITH_OVR option set to on.");
#endif
    }
    case TRACKER_VICON:
    {
#ifdef WITH_VICON
      m_viconTracker = new ViconTracker(m_trackerParams, "kinect");
      //m_tracker.reset(make_hybrid_tracker(m_viconTracker, settings, scene, trackedImageSize));
      m_tracker.reset(new RefineWithICPTracker(m_viconTracker, trackedImageSize, settings, m_lowLevelEngine, scene));
      break;
#else
      // This should never happen as things stand - we never try to use the Vicon tracker if Vicon support isn't available.
      throw std::runtime_error("Error: Vicon support not currently available. Reconfigure in CMake with the WITH_VICON option set to on.");
#endif
    }
    default: // TRACKER_INFINITAM
    {
      m_imuCalibrator.reset(new ITMIMUCalibrator_iPad);
      m_tracker.reset(ITMTrackerFactory<SpaintVoxel,ITMVoxelIndex>::Instance().Make(
        trackedImageSize, settings.get(), m_lowLevelEngine.get(), m_imuCalibrator.get(), scene.get()
      ));
    }
  }
}

}
