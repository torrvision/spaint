/**
 * spaintgui: Pipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Pipeline.h"
using namespace spaint;

#ifdef WITH_OPENCV
#include <spaint/ocv/OpenCVUtil.h>
#endif

//#################### CONSTRUCTORS ####################

Pipeline::Pipeline(const CompositeImageSourceEngine_Ptr& imageSourceEngine, const Settings_Ptr& settings, const std::string& resourcesDir,
                   const LabelManager_Ptr& labelManager, unsigned int seed, TrackerType trackerType, const std::string& trackerParams)
: m_mode(MODE_NORMAL),
  m_propagationComponent(imageSourceEngine->getDepthImageSize(), settings),
  m_semanticSegmentationComponent(imageSourceEngine->getDepthImageSize(), seed, settings, resourcesDir, labelManager->get_max_label_count()),
  m_slamComponent(imageSourceEngine, settings, trackerType, trackerParams),
  m_smoothingComponent(labelManager->get_max_label_count(), settings)
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
  Vector2i depthImageSize = m_slamComponent.get_input_raw_depth_image()->noDims;
  Vector2i rgbImageSize = m_slamComponent.get_input_rgb_image()->noDims;

  m_model.reset(new Model(m_slamComponent.get_scene(), rgbImageSize, depthImageSize, m_slamComponent.get_tracking_state(), settings, resourcesDir, labelManager));
  m_visualisationGenerator.reset(new VisualisationGenerator(m_model->get_visualisation_engine(), labelManager, m_model->get_settings()));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool Pipeline::get_fusion_enabled() const
{
  return m_slamComponent.get_fusion_enabled();
}

ITMShortImage_Ptr Pipeline::get_input_raw_depth_image_copy() const
{
  ITMShortImage_CPtr inputRawDepthImage = m_slamComponent.get_input_raw_depth_image();
  ITMShortImage_Ptr copy(new ITMShortImage(inputRawDepthImage->noDims, true, false));
  copy->SetFrom(inputRawDepthImage.get(), ORUtils::MemoryBlock<short>::CPU_TO_CPU);
  return copy;
}

ITMUChar4Image_Ptr Pipeline::get_input_rgb_image_copy() const
{
  ITMUChar4Image_CPtr inputRGBImage = m_slamComponent.get_input_rgb_image();
  ITMUChar4Image_Ptr copy(new ITMUChar4Image(inputRGBImage->noDims, true, false));
  copy->SetFrom(inputRGBImage.get(), ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
  return copy;
}

Pipeline::RenderState_CPtr Pipeline::get_live_render_state() const
{
  return m_slamComponent.get_live_render_state();
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

void Pipeline::reset_forest()
{
  m_semanticSegmentationComponent.reset_forest();
}

bool Pipeline::run_main_section()
{
  return m_slamComponent.run(*m_model);
}

void Pipeline::run_mode_specific_section(const RenderState_CPtr& renderState)
{
  switch(m_mode)
  {
    case MODE_FEATURE_INSPECTION:
      m_semanticSegmentationComponent.run_feature_inspection(*m_model, renderState);
      break;
    case MODE_PREDICTION:
      m_semanticSegmentationComponent.run_prediction(*m_model, renderState);
      break;
    case MODE_PROPAGATION:
      m_propagationComponent.run(*m_model, renderState);
      break;
    case MODE_SMOOTHING:
      m_smoothingComponent.run(*m_model, renderState);
      break;
    case MODE_TRAIN_AND_PREDICT:
    {
      static bool trainThisFrame = false;
      trainThisFrame = !trainThisFrame;

      if(trainThisFrame) m_semanticSegmentationComponent.run_training(*m_model, renderState);
      else m_semanticSegmentationComponent.run_prediction(*m_model, renderState);;

      break;
    }
    case MODE_TRAINING:
      m_semanticSegmentationComponent.run_training(*m_model, renderState);
      break;
    default:
      break;
  }
}

void Pipeline::set_fusion_enabled(bool fusionEnabled)
{
  m_slamComponent.set_fusion_enabled(fusionEnabled);
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
