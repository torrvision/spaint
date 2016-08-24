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

//#################### CONSTRUCTORS ####################

Pipeline::Pipeline(const CompositeImageSourceEngine_Ptr& imageSourceEngine, const Settings_Ptr& settings, const std::string& resourcesDir,
                   const LabelManager_Ptr& labelManager, unsigned int seed, TrackerType trackerType, const std::string& trackerParams)
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
  m_visualisationGenerator.reset(new VisualisationGenerator(m_model->get_visualisation_engine(), labelManager, settings));

  // Set up the pipeline components.
  const std::string worldSceneID = "World";
  m_slamComponent.reset(new SLAMComponent(m_model, worldSceneID, imageSourceEngine, trackerType, trackerParams));
  m_propagationComponent.reset(new PropagationComponent(m_model, worldSceneID));
  m_semanticSegmentationComponent.reset(new SemanticSegmentationComponent(m_model, worldSceneID, seed));
  m_smoothingComponent.reset(new SmoothingComponent(m_model, worldSceneID));

  // TEMPORARY
#if 0
  boost::shared_ptr<CompositeImageSourceEngine> objectImageSourceEngine(new CompositeImageSourceEngine);
  ImageMaskPathGenerator pathGenerator("C:/fr4_tsukuba/rgb_rnm/%06i.ppm", "C:/fr4_tsukuba/depth_rnm/%06i.pgm");
  objectImageSourceEngine->addSubengine(new ImageFileReader<ImageMaskPathGenerator>("C:/fr4_tsukuba/calibration.txt", pathGenerator, 0));
  const std::string objectSceneID = "Object";
  m_objectSlamComponent.reset(new SLAMComponent(m_model, objectSceneID, objectImageSourceEngine, trackerType, trackerParams));
#endif
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool Pipeline::get_fusion_enabled() const
{
  return m_slamComponent->get_fusion_enabled();
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

void Pipeline::reset_forest()
{
  m_semanticSegmentationComponent->reset_forest();
}

bool Pipeline::run_main_section()
{
#if 0
  m_objectSlamComponent->run();
#endif
  return m_slamComponent->run();
}

void Pipeline::run_mode_specific_section(const RenderState_CPtr& renderState)
{
  switch(m_mode)
  {
    case MODE_FEATURE_INSPECTION:
      m_semanticSegmentationComponent->run_feature_inspection(renderState);
      break;
    case MODE_PREDICTION:
      m_semanticSegmentationComponent->run_prediction(renderState);
      break;
    case MODE_PROPAGATION:
      m_propagationComponent->run(renderState);
      break;
    case MODE_SMOOTHING:
      m_smoothingComponent->run(renderState);
      break;
    case MODE_TRAIN_AND_PREDICT:
    {
      static bool trainThisFrame = false;
      trainThisFrame = !trainThisFrame;

      if(trainThisFrame) m_semanticSegmentationComponent->run_training(renderState);
      else m_semanticSegmentationComponent->run_prediction(renderState);

      break;
    }
    case MODE_TRAINING:
      m_semanticSegmentationComponent->run_training(renderState);
      break;
    default:
      break;
  }
}

void Pipeline::set_fusion_enabled(bool fusionEnabled)
{
  m_slamComponent->set_fusion_enabled(fusionEnabled);
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
