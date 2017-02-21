/**
 * spaintgui: SLAMPipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "SLAMPipeline.h"
using namespace spaint;

#ifdef WITH_OPENCV
#include <spaint/ocv/OpenCVUtil.h>
#endif

#include <spaint/pipelinecomponents/SLAMComponentWithScoreForest.h>

//#################### CONSTRUCTORS ####################

SLAMPipeline::SLAMPipeline(const Settings_Ptr& settings,
    const std::string& experimentTag,
    const std::string& resourcesDir,
    const CompositeImageSourceEngine_Ptr& imageSourceEngine,
    spaint::TrackerType trackerType,
    const std::vector<std::string>& trackerParams,
    spaint::SLAMComponent::MappingMode mappingMode,
    spaint::SLAMComponent::TrackingMode trackingMode)
: MultiScenePipeline("slam", settings, resourcesDir, 2, experimentTag)
// Need to use 2 labels to avoid crash.
// TODO fix it
// using 0 crashes for an invalid MemoryBlock allocation
// using 1 -> Application.cpp#715 needs at least 2 valid labels
{
  const std::string sceneID = Model::get_world_scene_id();
  m_slamComponents[sceneID].reset(new SLAMComponentWithScoreForest(
      m_model,
      sceneID,
      imageSourceEngine,
      trackerType,
      trackerParams,
      mappingMode,
      trackingMode
  ));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SLAMPipeline::set_mode(Mode mode)
{
  // The only supported mode
  m_mode = MODE_NORMAL;
}
