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
#include <ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
using namespace InputSource;
using namespace ITMLib;
using namespace ORUtils;
using namespace RelocLib;

#include <spaint/features/FeatureCalculatorFactory.h>
#include <spaint/randomforest/ForestUtil.h>
#include <spaint/randomforest/SpaintDecisionFunctionGenerator.h>
#include <spaint/util/MemoryBlockFactory.h>

#ifdef WITH_OPENCV
#include <spaint/ocv/OpenCVUtil.h>
#endif

#define DEBUGGING 1

//#################### CONSTRUCTORS ####################

Pipeline::Pipeline(const CompositeImageSourceEngine_Ptr& imageSourceEngine, const Settings_Ptr& settings, const std::string& resourcesDir,
                   const LabelManager_Ptr& labelManager, unsigned int seed, TrackerType trackerType, const std::string& trackerParams)
: m_mode(PIPELINEMODE_NORMAL),
  m_predictionSection(imageSourceEngine->getDepthImageSize(), seed, settings),
  m_propagationSection(imageSourceEngine->getDepthImageSize(), settings),
  m_resourcesDir(resourcesDir),
  m_slamSection(imageSourceEngine, settings, trackerType, trackerParams),
  m_smoothingSection(labelManager->get_max_label_count(), settings),
  m_trainingSection(imageSourceEngine->getDepthImageSize(), seed, settings, labelManager->get_max_label_count())
{
  // Make sure that we're not trying to run on the GPU if CUDA support isn't enabled.
#ifndef WITH_CUDA
  if(settings->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
    std::cerr << "[spaint] CUDA support unavailable, reverting to the CPU implementation of InfiniTAM\n";
    settings->deviceType = ITMLibSettings::DEVICE_CPU;
  }
#endif

  // Set up the spaint model, raycaster and interactor.
  Vector2i depthImageSize = m_slamSection.get_input_raw_depth_image()->noDims;
  Vector2i rgbImageSize = m_slamSection.get_input_rgb_image()->noDims;
  Vector2i trackedImageSize = m_slamSection.get_tracked_image_size(rgbImageSize, depthImageSize);

  m_state.m_model.reset(new Model(m_slamSection.get_scene(), rgbImageSize, depthImageSize, m_slamSection.get_tracking_state(), settings, resourcesDir, labelManager));
  m_state.m_raycaster.reset(new Raycaster(m_state.get_model(), trackedImageSize, settings));
  m_state.m_interactor.reset(new Interactor(m_state.get_model()));

  // Get the maximum numbers of voxels to use for prediction and training.
  const size_t maxPredictionVoxelCount = m_predictionSection.get_max_prediction_voxel_count();
  const size_t maxTrainingVoxelCount = m_trainingSection.get_max_training_voxel_count();

  // Set up the feature calculator.
  // FIXME: These values shouldn't be hard-coded here ultimately.
  m_state.m_patchSize = 13;
  const float patchSpacing = 0.01f / settings->sceneParams.voxelSize; // 10mm = 0.01m (dividing by the voxel size, which is in m, expresses the spacing in voxels)
  const size_t binCount = 36;                                         // 10 degrees per bin

  m_state.m_featureCalculator = FeatureCalculatorFactory::make_vop_feature_calculator(
    std::max(maxPredictionVoxelCount, maxTrainingVoxelCount),
    m_state.m_patchSize, patchSpacing, binCount, settings->deviceType
  );

  // Set up the memory blocks needed to store the features computed during prediction and training.
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
  const size_t featureCount = m_state.m_featureCalculator->get_feature_count();
  m_state.m_predictionFeaturesMB = mbf.make_block<float>(maxPredictionVoxelCount * featureCount);
  m_state.m_trainingFeaturesMB = mbf.make_block<float>(maxTrainingVoxelCount * featureCount);

  // Register the relevant decision function generators with the factory.
  DecisionFunctionGeneratorFactory<SpaintVoxel::Label>::instance().register_maker(
    SpaintDecisionFunctionGenerator::get_static_type(),
    &SpaintDecisionFunctionGenerator::maker
  );

  // Set up the random forest.
  reset_forest();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool Pipeline::get_fusion_enabled() const
{
  return m_slamSection.get_fusion_enabled();
}

ITMShortImage_Ptr Pipeline::get_input_raw_depth_image_copy() const
{
  ITMShortImage_CPtr inputRawDepthImage = m_slamSection.get_input_raw_depth_image();
  ITMShortImage_Ptr copy(new ITMShortImage(inputRawDepthImage->noDims, true, false));
  copy->SetFrom(inputRawDepthImage.get(), ORUtils::MemoryBlock<short>::CPU_TO_CPU);
  return copy;
}

ITMUChar4Image_Ptr Pipeline::get_input_rgb_image_copy() const
{
  ITMUChar4Image_CPtr inputRGBImage = m_slamSection.get_input_rgb_image();
  ITMUChar4Image_Ptr copy(new ITMUChar4Image(inputRGBImage->noDims, true, false));
  copy->SetFrom(inputRGBImage.get(), ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
  return copy;
}

const Interactor_Ptr& Pipeline::get_interactor()
{
  return m_state.m_interactor;
}

Pipeline::RenderState_CPtr Pipeline::get_live_render_state() const
{
  return m_slamSection.get_live_render_state();
}

PipelineMode Pipeline::get_mode() const
{
  return m_mode;
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
  DecisionTree<SpaintVoxel::Label>::Settings dtSettings(m_resourcesDir + "/RaflSettings.xml");
  m_state.m_forest.reset(new RandomForest<SpaintVoxel::Label>(treeCount, dtSettings));
}

bool Pipeline::run_main_section()
{
  return m_slamSection.run(m_state);
}

void Pipeline::run_mode_specific_section(const RenderState_CPtr& renderState)
{
  switch(m_mode)
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
  m_slamSection.set_fusion_enabled(fusionEnabled);
}

void Pipeline::set_mode(PipelineMode mode)
{
#ifdef WITH_OPENCV
  // If we are switching out of feature inspection mode, destroy the feature inspection window.
  if(m_mode == PIPELINEMODE_FEATURE_INSPECTION && mode != PIPELINEMODE_FEATURE_INSPECTION)
  {
    cv::destroyAllWindows();
  }
#endif

  m_mode = mode;
}
