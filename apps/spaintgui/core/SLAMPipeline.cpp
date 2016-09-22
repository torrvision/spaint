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

SLAMPipeline::SLAMPipeline(const Settings_Ptr& settings,
    const std::string& resourcesDir,
    const CompositeImageSourceEngine_Ptr& imageSourceEngine,
    spaint::TrackerType trackerType,
    const std::vector<std::string>& trackerParams,
    spaint::SLAMComponent::MappingMode mappingMode,
    spaint::SLAMComponent::TrackingMode trackingMode)
: MultiScenePipeline(settings, resourcesDir, 2)
// Need to use 2 labels to avoid crash.
// TODO fix it
// using 0 crashes for an invalid MemoryBlock allocation
// using 1 -> Application.cpp#715 needs at least 2 valid labels
{
  const std::string sceneID = Model::get_world_scene_id();
  m_slamComponents[sceneID].reset(new SLAMComponent(m_model, sceneID, imageSourceEngine, trackerType, trackerParams, mappingMode, trackingMode));
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
