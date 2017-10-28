/**
 * spaintgui: CollaborativePipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "CollaborativePipeline.h"
using namespace itmx;
using namespace spaint;

#include "itmx/imagesources/RemoteImageSourceEngine.h"

//#################### CONSTRUCTORS ####################

CollaborativePipeline::CollaborativePipeline(const Settings_Ptr& settings, const std::string& resourcesDir,
                                             const std::vector<CompositeImageSourceEngine_Ptr>& imageSourceEngines,
                                             const std::vector<std::string>& trackerConfigs,
                                             const std::vector<SLAMComponent::MappingMode>& mappingModes,
                                             const std::vector<SLAMComponent::TrackingMode>& trackingModes,
                                             const FiducialDetector_CPtr& fiducialDetector, bool detectFiducials,
                                             const MappingServer_Ptr& mappingServer, CollaborationMode collaborationMode)
  // Note: A minimum of 2 labels is required (background and foreground).
: MultiScenePipeline("collaborative", settings, resourcesDir, 2, mappingServer),
  m_collaborationStarted(false),
  m_worldIsRemote(imageSourceEngines.empty())
{
  if(imageSourceEngines.empty())
  {
    // The current implementation of spaintgui relies on the existence of a scene called "World".
    // If no local scenes were specified when the pipeline was instantiated, then we check to
    // see if a mapping server exists:
    if(mappingServer)
    {
      // If it does, we wait for the first remote client to join instead, and call its scene "World" when it
      // connects. The creation of the SLAM component for the remote client will block until it does so.
      add_remote_slam_component(Model::get_world_scene_id(), 0);
    }
    else
    {
      // Otherwise, no scene called "World" will ever exist, so we throw.
      throw std::runtime_error("Error: Cannot run a collaborative pipeline without a scene called 'World' (did you mean to specify --runServer?)");
    }
  }
  else
  {
    // If local scenes were specified when the pipeline was instantiated, we add a SLAM component for each such scene.
    for(size_t i = 0, size = imageSourceEngines.size(); i < size; ++i)
    {
      const std::string sceneID = i == 0 ? Model::get_world_scene_id() : "Local" + boost::lexical_cast<std::string>(i);
      m_slamComponents[sceneID].reset(new SLAMComponent(m_model, sceneID, imageSourceEngines[i], trackerConfigs[i], mappingModes[i], trackingModes[i], fiducialDetector, detectFiducials));
    }
  }

  // Finally, we add a collaborative component to handle relocalisation between the different scenes.
  m_collaborativeComponent.reset(new CollaborativeComponent(m_model, collaborationMode));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

size_t CollaborativePipeline::run_main_section()
{
  // If we're running a mapping server, add SLAM components for any newly-connected remote clients.
  if(m_model->get_mapping_server()) check_for_new_clients();

  // Run the main section of the pipeline.
  size_t scenesFused = MultiScenePipeline::run_main_section();

  // Provided at least one of the scenes has started fusion, run the collaborative pose estimation process.
  m_collaborationStarted = m_collaborationStarted || scenesFused > 0;
  if(m_collaborationStarted) m_collaborativeComponent->run_collaborative_pose_estimation();

  return scenesFused;
}

void CollaborativePipeline::set_mode(Mode mode)
{
  // The only supported mode.
  m_mode = MODE_NORMAL;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void CollaborativePipeline::add_remote_slam_component(const std::string& sceneID, int remoteClientID)
{
  std::cout << "Instantiating a SLAM component for client '" << remoteClientID << "', with local scene ID '" << sceneID << "'" << std::endl;

  // Note: We create remote clients that are voxel-only and have no support for fiducials.
  const ImageSourceEngine_Ptr imageSourceEngine(new RemoteImageSourceEngine(m_model->get_mapping_server(), remoteClientID));
  const std::string trackerConfig = "<tracker type='remote'><params>" + boost::lexical_cast<std::string>(remoteClientID) + "</params></tracker>";
  const SLAMComponent::MappingMode mappingMode = SLAMComponent::MappingMode::MAP_VOXELS_ONLY;
  const SLAMComponent::TrackingMode trackingMode = SLAMComponent::TrackingMode::TRACK_VOXELS;
  const FiducialDetector_CPtr fiducialDetector;
  const bool detectFiducials = false;
  m_slamComponents[sceneID].reset(new SLAMComponent(m_model, sceneID, imageSourceEngine, trackerConfig, mappingMode, trackingMode, fiducialDetector, detectFiducials));
}

void CollaborativePipeline::check_for_new_clients()
{
  // Check if there are new remote clients, and add SLAM components for them if so.
  const std::vector<int> activeClients = m_model->get_mapping_server()->get_active_clients();
  for(size_t i = 0, size = activeClients.size(); i < size; ++i)
  {
    const int clientID = activeClients[i];

    // Determine what the scene for this client should be called.
    const std::string sceneID = m_worldIsRemote && clientID == 0 ? Model::get_world_scene_id() : "Remote" + boost::lexical_cast<std::string>(clientID);

    // If a SLAM component for this client does not yet exist, add one now.
    if(m_slamComponents.find(sceneID) == m_slamComponents.end())
    {
      add_remote_slam_component(sceneID, clientID);
    }
  }
}
