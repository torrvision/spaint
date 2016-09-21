/**
 * spaintgui: SemanticPipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "SemanticPipeline.h"
using namespace spaint;

#ifdef WITH_OPENCV
#include <spaint/ocv/OpenCVUtil.h>
#endif

//#################### CONSTRUCTORS ####################

SemanticPipeline::SemanticPipeline(const Settings_Ptr& settings, const std::string& resourcesDir, size_t maxLabelCount,
                                   const CompositeImageSourceEngine_Ptr& imageSourceEngine, unsigned int seed,
                                   TrackerType trackerType, const std::string& trackerParams,
                                   SLAMComponent::MappingMode mappingMode, SLAMComponent::TrackingMode trackingMode,
                                   const FiducialDetector_CPtr& fiducialDetector)
: MultiScenePipeline(settings, resourcesDir, maxLabelCount)
{
  const std::string sceneID = Model::get_world_scene_id();
  m_slamComponents[sceneID].reset(new SLAMComponent(m_model, sceneID, imageSourceEngine, trackerType, trackerParams, mappingMode, trackingMode, fiducialDetector));
  m_propagationComponents[sceneID].reset(new PropagationComponent(m_model, sceneID));
  m_semanticSegmentationComponents[sceneID].reset(new SemanticSegmentationComponent(m_model, sceneID, seed));
  m_smoothingComponents[sceneID].reset(new SmoothingComponent(m_model, sceneID));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SemanticPipeline::set_mode(Mode mode)
{
#ifdef WITH_OPENCV
  // If we are switching out of feature inspection mode, destroy the feature inspection window.
  if(m_mode == MODE_FEATURE_INSPECTION && mode != MODE_FEATURE_INSPECTION)
  {
    cv::destroyAllWindows();
  }
#endif

  switch(mode)
  {
    case MODE_FEATURE_INSPECTION:
    case MODE_NORMAL:
    case MODE_PREDICTION:
    case MODE_PROPAGATION:
    case MODE_SMOOTHING:
    case MODE_TRAINING:
    case MODE_TRAIN_AND_PREDICT:
      m_mode = mode;
      break;
    default:
      break;
  }
}
