/**
 * spaintgui: Pipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Pipeline.h"
using namespace spaint;

#include <InputSource/CompositeImageSourceEngine.h>
using namespace InputSource;

#ifdef WITH_OPENCV
#include <spaint/ocv/OpenCVUtil.h>
#endif

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

//#################### CONSTRUCTORS ####################

Pipeline::Pipeline(const Settings_Ptr& settings, const std::string& resourcesDir, const LabelManager_Ptr& labelManager)
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

  // Set up the spaint model and visualisation generator.
  m_model.reset(new Model(settings, resourcesDir, labelManager));
  m_visualisationGenerator.reset(new VisualisationGenerator(m_model->get_visualisation_engine(), m_model->get_surfel_visualisation_engine(), labelManager, settings));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void Pipeline::add_scene_pipeline(const std::string& sceneID, const CompositeImageSourceEngine_Ptr& imageSourceEngine, unsigned int seed,
                                  spaint::TrackerType trackerType, const std::string& trackerParams, SLAMComponent::MappingMode mappingMode)
{
  ScenePipeline& scenePipeline = m_scenePipelines[sceneID];
  scenePipeline.m_slamComponent.reset(new SLAMComponent(m_model, sceneID, imageSourceEngine, trackerType, trackerParams, mappingMode));
  scenePipeline.m_propagationComponent.reset(new PropagationComponent(m_model, sceneID));
  scenePipeline.m_semanticSegmentationComponent.reset(new SemanticSegmentationComponent(m_model, sceneID, seed));
  scenePipeline.m_smoothingComponent.reset(new SmoothingComponent(m_model, sceneID));
}

bool Pipeline::get_fusion_enabled(const std::string& sceneID) const
{
  return MapUtil::lookup(m_scenePipelines, sceneID).m_slamComponent->get_fusion_enabled();
}

ITMShortImage_Ptr Pipeline::get_input_raw_depth_image_copy(const std::string& sceneID) const
{
  ITMShortImage_CPtr inputRawDepthImage = m_model->get_input_raw_depth_image(sceneID);
  ITMShortImage_Ptr copy(new ITMShortImage(inputRawDepthImage->noDims, true, false));
  copy->SetFrom(inputRawDepthImage.get(), ORUtils::MemoryBlock<short>::CPU_TO_CPU);
  return copy;
}

ITMUChar4Image_Ptr Pipeline::get_input_rgb_image_copy(const std::string& sceneID) const
{
  ITMUChar4Image_CPtr inputRGBImage = m_model->get_input_rgb_image(sceneID);
  ITMUChar4Image_Ptr copy(new ITMUChar4Image(inputRGBImage->noDims, true, false));
  copy->SetFrom(inputRGBImage.get(), ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
  return copy;
}

Pipeline::Mode Pipeline::get_mode() const
{
  return m_mode;
}

const Model_Ptr& Pipeline::get_model()
{
  return m_model;
}

Model_CPtr Pipeline::get_model() const
{
  return m_model;
}

VisualisationGenerator_CPtr Pipeline::get_visualisation_generator() const
{
  return m_visualisationGenerator;
}

void Pipeline::reset_forest(const std::string& sceneID)
{
  MapUtil::lookup(m_scenePipelines, sceneID).m_semanticSegmentationComponent->reset_forest();
}

bool Pipeline::run_main_section()
{
  bool result = true;
  for(std::map<std::string,ScenePipeline>::const_iterator it = m_scenePipelines.begin(), iend = m_scenePipelines.end(); it != iend; ++it)
  {
    result = result && it->second.m_slamComponent->run();
  }
  return result;
}

void Pipeline::run_mode_specific_section(const std::string& sceneID, const RenderState_CPtr& renderState)
{
  ScenePipeline& scenePipeline = MapUtil::lookup(m_scenePipelines, sceneID);

  switch(m_mode)
  {
    case MODE_FEATURE_INSPECTION:
      scenePipeline.m_semanticSegmentationComponent->run_feature_inspection(renderState);
      break;
    case MODE_PREDICTION:
      scenePipeline.m_semanticSegmentationComponent->run_prediction(renderState);
      break;
    case MODE_PROPAGATION:
      scenePipeline.m_propagationComponent->run(renderState);
      break;
    case MODE_SMOOTHING:
      scenePipeline.m_smoothingComponent->run(renderState);
      break;
    case MODE_TRAIN_AND_PREDICT:
    {
      static bool trainThisFrame = false;
      trainThisFrame = !trainThisFrame;

      if(trainThisFrame) scenePipeline.m_semanticSegmentationComponent->run_training(renderState);
      else scenePipeline.m_semanticSegmentationComponent->run_prediction(renderState);

      break;
    }
    case MODE_TRAINING:
      scenePipeline.m_semanticSegmentationComponent->run_training(renderState);
      break;
    default:
      break;
  }
}

void Pipeline::set_fusion_enabled(const std::string& sceneID, bool fusionEnabled)
{
  MapUtil::lookup(m_scenePipelines, sceneID).m_slamComponent->set_fusion_enabled(fusionEnabled);
}

void Pipeline::set_mode(Mode mode)
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
