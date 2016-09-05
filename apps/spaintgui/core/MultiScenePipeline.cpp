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

//#################### PUBLIC MEMBER FUNCTIONS ####################

void MultiScenePipeline::add_semantic_components(const std::string& sceneID, const CompositeImageSourceEngine_Ptr& imageSourceEngine,
                                                 unsigned int seed, TrackerType trackerType, const std::string& trackerParams,
                                                 SLAMComponent::MappingMode mappingMode, SLAMComponent::TrackingMode trackingMode)
{
  m_slamComponents[sceneID].reset(new SLAMComponent(m_model, sceneID, imageSourceEngine, trackerType, trackerParams, mappingMode, trackingMode));
  m_objectSegmentationComponents[sceneID].reset(new ObjectSegmentationComponent(m_model, sceneID));
  m_propagationComponents[sceneID].reset(new PropagationComponent(m_model, sceneID));
  m_semanticSegmentationComponents[sceneID].reset(new SemanticSegmentationComponent(m_model, sceneID, seed));
  m_smoothingComponents[sceneID].reset(new SmoothingComponent(m_model, sceneID));
}

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
  MapUtil::lookup(m_semanticSegmentationComponents, sceneID)->reset_forest();
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
      MapUtil::lookup(m_semanticSegmentationComponents, sceneID)->run_feature_inspection(renderState);
      break;
    case MODE_PREDICTION:
      MapUtil::lookup(m_semanticSegmentationComponents, sceneID)->run_prediction(renderState);
      break;
    case MODE_PROPAGATION:
      MapUtil::lookup(m_propagationComponents, sceneID)->run(renderState);
      break;
    case MODE_SEGMENTATION:
      MapUtil::lookup(m_objectSegmentationComponents, sceneID)->run_segmentation(renderState);
      break;
    case MODE_SEGMENTATION_TRAINING:
      MapUtil::lookup(m_objectSegmentationComponents, sceneID)->run_segmentation_training(renderState);
      break;
    case MODE_SMOOTHING:
      MapUtil::lookup(m_smoothingComponents, sceneID)->run(renderState);
      break;
    case MODE_TRAIN_AND_PREDICT:
    {
      static bool trainThisFrame = false;
      trainThisFrame = !trainThisFrame;

      if(trainThisFrame) MapUtil::lookup(m_semanticSegmentationComponents, sceneID)->run_training(renderState);
      else MapUtil::lookup(m_semanticSegmentationComponents, sceneID)->run_prediction(renderState);

      break;
    }
    case MODE_TRAINING:
      MapUtil::lookup(m_semanticSegmentationComponents, sceneID)->run_training(renderState);
      break;
    default:
      break;
  }
}

void MultiScenePipeline::set_fusion_enabled(const std::string& sceneID, bool fusionEnabled)
{
  MapUtil::lookup(m_slamComponents, sceneID)->set_fusion_enabled(fusionEnabled);
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

  // If we are switching into segmentation training mode, reset the segmenter.
  if(mode == MODE_SEGMENTATION_TRAINING && m_mode != MODE_SEGMENTATION_TRAINING)
  {
    MapUtil::lookup(m_objectSegmentationComponents, Model::get_world_scene_id())->reset_segmenter();
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
