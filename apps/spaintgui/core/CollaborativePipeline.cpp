/**
 * spaintgui: CollaborativePipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "CollaborativePipeline.h"
using namespace itmx;
using namespace spaint;

//#################### CONSTRUCTORS ####################

CollaborativePipeline::CollaborativePipeline(const Settings_Ptr& settings, const std::string& resourcesDir,
                                             const std::vector<CompositeImageSourceEngine_Ptr>& imageSourceEngines,
                                             const std::vector<std::string>& trackerConfigs,
                                             const std::vector<SLAMComponent::MappingMode>& mappingModes,
                                             const std::vector<SLAMComponent::TrackingMode>& trackingModes,
                                             const FiducialDetector_CPtr& fiducialDetector, bool detectFiducials,
                                             const MappingServer_Ptr& mappingServer)
  // Note: A minimum of 2 labels is required (background and foreground).
: MultiScenePipeline("collaborative", settings, resourcesDir, 2, mappingServer), m_worldFusionStarted(false)
{
  for(size_t i = 0, size = imageSourceEngines.size(); i < size; ++i)
  {
    const std::string sceneID = i == 0 ? Model::get_world_scene_id() : "Agent" + boost::lexical_cast<std::string>(i);
    m_slamComponents[sceneID].reset(new SLAMComponent(m_model, sceneID, imageSourceEngines[i], trackerConfigs[i], mappingModes[i], trackingModes[i], fiducialDetector, detectFiducials));
  }

  m_collaborativeComponent.reset(new CollaborativeComponent(m_model));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

size_t CollaborativePipeline::run_main_section()
{
  size_t scenesFused = MultiScenePipeline::run_main_section();
  m_worldFusionStarted = m_worldFusionStarted || scenesFused == m_slamComponents.size();
  if(m_worldFusionStarted) m_collaborativeComponent->run_collaborative_pose_estimation();
  return scenesFused;
}

void CollaborativePipeline::set_mode(Mode mode)
{
  // The only supported mode.
  m_mode = MODE_NORMAL;
}
