/**
 * spaintgui: SLAMPipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "SLAMPipeline.h"
using namespace spaint;

#ifdef WITH_OPENCV
#include <spaint/ocv/OpenCVUtil.h>
#endif

//#################### CONSTRUCTORS ####################

SLAMPipeline::SLAMPipeline(const Settings_Ptr& settings, const std::string& resourcesDir, size_t maxLabelCount,
                                   const CompositeImageSourceEngine_Ptr& imageSourceEngine, unsigned int seed,
                                   TrackerType trackerType, const std::vector<std::string>& trackerParams,
                                   SLAMComponent::MappingMode mappingMode, SLAMComponent::TrackingMode trackingMode)
: MultiScenePipeline(settings, resourcesDir, maxLabelCount)
{
  const std::string sceneID = Model::get_world_scene_id();
  m_slamComponents[sceneID].reset(new SLAMComponent(m_model, sceneID, imageSourceEngine, trackerType, trackerParams, mappingMode, trackingMode));
  m_propagationComponents[sceneID].reset(new PropagationComponent(m_model, sceneID));
  m_semanticSegmentationComponents[sceneID].reset(new SemanticSegmentationComponent(m_model, sceneID, seed));
  m_smoothingComponents[sceneID].reset(new SmoothingComponent(m_model, sceneID));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SLAMPipeline::set_mode(Mode mode)
{
#ifdef WITH_OPENCV
  // If we are switching out of feature inspection mode, destroy the feature inspection window.
  if(m_mode == MODE_FEATURE_INSPECTION && mode != MODE_FEATURE_INSPECTION)
  {
    cv::destroyAllWindows();
  }
#endif

  // The only supported mode
  m_mode = MODE_NORMAL;
}
