/**
 * spaintgui: MultiScenePipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "MultiScenePipeline.h"
using namespace InputSource;
using namespace ITMLib;
using namespace spaint;

#include <boost/bind.hpp>

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

//#################### DESTRUCTOR ####################

MultiScenePipeline::~MultiScenePipeline() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool MultiScenePipeline::get_fusion_enabled(const std::string& sceneID) const
{
  return MapUtil::lookup(m_slamComponents, sceneID)->get_fusion_enabled();
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
  MapUtil::call_if_found(m_semanticSegmentationComponents, sceneID, boost::bind(&SemanticSegmentationComponent::reset_forest, _1));
}

bool MultiScenePipeline::run_main_section()
{
  bool result = true;
  for(std::map<std::string,SLAMComponent_Ptr>::const_iterator it = m_slamComponents.begin(), iend = m_slamComponents.end(); it != iend; ++it)
  {
    result = result && it->second->process_frame();
  }
  return result;
}

void MultiScenePipeline::run_mode_specific_section(const std::string& sceneID, const VoxelRenderState_CPtr& renderState)
{
  switch(m_mode)
  {
    case MODE_FEATURE_INSPECTION:
      MapUtil::call_if_found(m_semanticSegmentationComponents, sceneID, boost::bind(&SemanticSegmentationComponent::run_feature_inspection, _1, renderState));
      break;
    case MODE_PREDICTION:
      MapUtil::call_if_found(m_semanticSegmentationComponents, sceneID, boost::bind(&SemanticSegmentationComponent::run_prediction, _1, renderState));
      break;
    case MODE_PROPAGATION:
      MapUtil::call_if_found(m_propagationComponents, sceneID, boost::bind(&PropagationComponent::run, _1, renderState));
      break;
    case MODE_SEGMENTATION:
      MapUtil::call_if_found(m_objectSegmentationComponents, sceneID, boost::bind(&ObjectSegmentationComponent::run_segmentation, _1, renderState));
      break;
    case MODE_SEGMENTATION_TRAINING:
      MapUtil::call_if_found(m_objectSegmentationComponents, sceneID, boost::bind(&ObjectSegmentationComponent::run_segmentation_training, _1, renderState));
      break;
    case MODE_SMOOTHING:
      MapUtil::call_if_found(m_smoothingComponents, sceneID, boost::bind(&SmoothingComponent::run, _1, renderState));
      break;
    case MODE_TRAIN_AND_PREDICT:
    {
      static bool trainThisFrame = false;
      trainThisFrame = !trainThisFrame;

      if(trainThisFrame) MapUtil::call_if_found(m_semanticSegmentationComponents, sceneID, boost::bind(&SemanticSegmentationComponent::run_training, _1, renderState));
      else MapUtil::call_if_found(m_semanticSegmentationComponents, sceneID, boost::bind(&SemanticSegmentationComponent::run_prediction, _1, renderState));

      break;
    }
    case MODE_TRAINING:
      MapUtil::call_if_found(m_semanticSegmentationComponents, sceneID, boost::bind(&SemanticSegmentationComponent::run_training, _1, renderState));
      break;
    default:
      break;
  }
}

void MultiScenePipeline::set_fusion_enabled(const std::string& sceneID, bool fusionEnabled)
{
  MapUtil::lookup(m_slamComponents, sceneID)->set_fusion_enabled(fusionEnabled);
}
