/**
 * spaintgui: MultiScenePipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "MultiScenePipeline.h"
using namespace InputSource;
using namespace ITMLib;
using namespace spaint;

#ifdef WITH_OPENCV
#include <spaint/ocv/OpenCVUtil.h>
#endif

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

//#################### CONSTRUCTORS ####################

MultiScenePipeline::MultiScenePipeline(const Settings_Ptr& settings, const std::string& resourcesDir, size_t maxLabelCount)
: m_mode(MODE_NORMAL)
{
  // Make sure that we're not trying to run on the GPU if CUDA support isn't enabled.
#ifndef WITH_CUDA
  if(settings->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
    std::cerr << "[spaint] CUDA support unavailable, reverting to the CPU implementation of InfiniTAM\n";
    settings->deviceType = ITMLibSettings::DEVICE_CPU;
  }
#endif

  // Set up the spaint model.
  m_model.reset(new Model(settings, resourcesDir, maxLabelCount));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void MultiScenePipeline::add_single_scene_pipeline(const std::string& sceneID, const CompositeImageSourceEngine_Ptr& imageSourceEngine,
                                                   unsigned int seed, TrackerType trackerType, const std::string& trackerParams,
                                                   SLAMComponent::MappingMode mappingMode, SLAMComponent::TrackingMode trackingMode)
{
  SingleScenePipeline& singleScenePipeline = m_singleScenePipelines[sceneID];
  singleScenePipeline.m_slamComponent.reset(new SLAMComponent(m_model, sceneID, imageSourceEngine, trackerType, trackerParams, mappingMode, trackingMode));
  singleScenePipeline.m_propagationComponent.reset(new PropagationComponent(m_model, sceneID));
  singleScenePipeline.m_semanticSegmentationComponent.reset(new SemanticSegmentationComponent(m_model, sceneID, seed));
  singleScenePipeline.m_smoothingComponent.reset(new SmoothingComponent(m_model, sceneID));
}

bool MultiScenePipeline::get_fusion_enabled(const std::string& sceneID) const
{
  return MapUtil::lookup(m_singleScenePipelines, sceneID).m_slamComponent->get_fusion_enabled();
}

ITMShortImage_Ptr MultiScenePipeline::get_input_raw_depth_image_copy(const std::string& sceneID) const
{
  ITMShortImage_CPtr inputRawDepthImage = m_model->get_input_raw_depth_image(sceneID);
  ITMShortImage_Ptr copy(new ITMShortImage(inputRawDepthImage->noDims, true, false));
  copy->SetFrom(inputRawDepthImage.get(), ORUtils::MemoryBlock<short>::CPU_TO_CPU);
  return copy;
}

ITMUChar4Image_Ptr MultiScenePipeline::get_input_rgb_image_copy(const std::string& sceneID) const
{
  ITMUChar4Image_CPtr inputRGBImage = m_model->get_input_rgb_image(sceneID);
  ITMUChar4Image_Ptr copy(new ITMUChar4Image(inputRGBImage->noDims, true, false));
  copy->SetFrom(inputRGBImage.get(), ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
  return copy;
}

MultiScenePipeline::Mode MultiScenePipeline::get_mode() const
{
  return m_mode;
}

const Model_Ptr& MultiScenePipeline::get_model()
{
  return m_model;
}

Model_CPtr MultiScenePipeline::get_model() const
{
  return m_model;
}

void MultiScenePipeline::reset_forest(const std::string& sceneID)
{
  MapUtil::lookup(m_singleScenePipelines, sceneID).m_semanticSegmentationComponent->reset_forest();
}

bool MultiScenePipeline::run_main_section()
{
  bool result = true;
  for(std::map<std::string,SingleScenePipeline>::const_iterator it = m_singleScenePipelines.begin(), iend = m_singleScenePipelines.end(); it != iend; ++it)
  {
    result = result && it->second.m_slamComponent->process_frame();
  }
  return result;
}

void MultiScenePipeline::run_mode_specific_section(const std::string& sceneID, const VoxelRenderState_CPtr& renderState)
{
  SingleScenePipeline& singleScenePipeline = MapUtil::lookup(m_singleScenePipelines, sceneID);

  switch(m_mode)
  {
    case MODE_FEATURE_INSPECTION:
      singleScenePipeline.m_semanticSegmentationComponent->run_feature_inspection(renderState);
      break;
    case MODE_PREDICTION:
      singleScenePipeline.m_semanticSegmentationComponent->run_prediction(renderState);
      break;
    case MODE_PROPAGATION:
      singleScenePipeline.m_propagationComponent->run(renderState);
      break;
    case MODE_SMOOTHING:
      singleScenePipeline.m_smoothingComponent->run(renderState);
      break;
    case MODE_TRAIN_AND_PREDICT:
    {
      static bool trainThisFrame = false;
      trainThisFrame = !trainThisFrame;

      if(trainThisFrame) singleScenePipeline.m_semanticSegmentationComponent->run_training(renderState);
      else singleScenePipeline.m_semanticSegmentationComponent->run_prediction(renderState);

      break;
    }
    case MODE_TRAINING:
      singleScenePipeline.m_semanticSegmentationComponent->run_training(renderState);
      break;
    default:
      break;
  }
}

void MultiScenePipeline::set_fusion_enabled(const std::string& sceneID, bool fusionEnabled)
{
  MapUtil::lookup(m_singleScenePipelines, sceneID).m_slamComponent->set_fusion_enabled(fusionEnabled);
}

void MultiScenePipeline::set_mode(Mode mode)
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
