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
#include <tvgutil/filesystem/PathFinder.h>
#include <tvgutil/timing/TimeUtil.h>
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

#if 0
void MultiScenePipeline::set_mode(Mode mode)
{
#ifdef WITH_OPENCV
  // If we are switching out of feature inspection mode, destroy the feature inspection window.
  if(m_mode == MODE_FEATURE_INSPECTION && mode != MODE_FEATURE_INSPECTION)
  {
    cv::destroyAllWindows();
  }
#endif

  // If we are switching into segmentation training mode, reset the segmenter.
  if(mode == MODE_SEGMENTATION_TRAINING && m_mode != MODE_SEGMENTATION_TRAINING)
  {
    MapUtil::call_if_found(m_objectSegmentationComponents, Model::get_world_scene_id(), boost::bind(&ObjectSegmentationComponent::reset_segmenter, _1));
  }

  // If we are switching out of segmentation training mode, clear the segmentation image.
  if(m_mode == MODE_SEGMENTATION_TRAINING && mode != MODE_SEGMENTATION_TRAINING)
  {
    m_model->set_segmentation_image(ITMUChar4Image_CPtr());
  }

  // If we are switching into segmentation mode, start a new segmentation video.
  boost::optional<SequentialPathGenerator>& segmentationPathGenerator = m_model->get_segmentation_path_generator();
  if(mode == MODE_SEGMENTATION && m_mode != MODE_SEGMENTATION)
  {
    segmentationPathGenerator.reset(SequentialPathGenerator(find_subdir_from_executable("segmentations") / TimeUtil::get_iso_timestamp()));
    boost::filesystem::create_directories(segmentationPathGenerator->get_base_dir());
  }

  // If we are switching out of segmentation mode, stop recording the segmentation video
  // and clear the segmentation image and target mask.
  if(m_mode == MODE_SEGMENTATION && mode != MODE_SEGMENTATION)
  {
    segmentationPathGenerator.reset();
    m_model->set_segmentation_image(ITMUChar4Image_CPtr());
    m_model->get_segmenter()->get_target_mask().reset();
  }

  m_mode = mode;
}
#endif
