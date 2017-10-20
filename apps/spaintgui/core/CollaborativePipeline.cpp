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
: MultiScenePipeline("collaborative", settings, resourcesDir, 2, mappingServer), m_collaborationStarted(false), m_worldIsRemote(false)
{
  if(imageSourceEngines.empty())
  {
    m_worldIsRemote = true;
    create_remote_slam_component(Model::get_world_scene_id(), 0); // This will block until a client connects.
  }

  for(size_t i = 0, size = imageSourceEngines.size(); i < size; ++i)
  {
    const std::string sceneID = i == 0 ? Model::get_world_scene_id() : "Local" + boost::lexical_cast<std::string>(i);
    m_slamComponents[sceneID].reset(new SLAMComponent(m_model, sceneID, imageSourceEngines[i], trackerConfigs[i], mappingModes[i], trackingModes[i], fiducialDetector, detectFiducials));
  }

  m_collaborativeComponent.reset(new CollaborativeComponent(m_model, collaborationMode));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

size_t CollaborativePipeline::run_main_section()
{
  // Check if there are new active clients, and if so instantiate new SLAMComponents.
  const MappingServer_Ptr mappingServer = m_model->get_mapping_server();
  const std::vector<int> activeClients = mappingServer->get_active_clients();

  for(size_t i = 0; i < activeClients.size(); ++i)
  {
    const int clientID = activeClients[i];

    if(m_worldIsRemote && clientID == 0)
    {
      // If there were no local image sources we are using the first client to connect as World scene, that means that we have to ignore it here.
      continue;
    }

    const std::string sceneID = "Agent" + boost::lexical_cast<std::string>(clientID);

    if(m_slamComponents.find(sceneID) == m_slamComponents.end())
    {
      create_remote_slam_component(sceneID, clientID);
    }
  }

  size_t scenesFused = MultiScenePipeline::run_main_section();
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

void CollaborativePipeline::create_remote_slam_component(const std::string& localSceneID, int remoteClientID)
{
  std::cout << "Instantiating SLAMComponent for client: " << remoteClientID << ", with local ID: " << localSceneID << '\n';

  const MappingServer_Ptr mappingServer = m_model->get_mapping_server();

  // Setup needed variables.
  const RemoteImageSourceEngine_Ptr imageSource(new RemoteImageSourceEngine(mappingServer, remoteClientID));
  const std::string trackerConfig = "<tracker type='remote'><params>" + boost::lexical_cast<std::string>(remoteClientID) + "</params></tracker>";

  // Remote Clients are voxel-only and have no support for fiducials.
  const SLAMComponent::MappingMode mappingMode = SLAMComponent::MappingMode::MAP_VOXELS_ONLY;
  const SLAMComponent::TrackingMode trackingMode = SLAMComponent::TrackingMode::TRACK_VOXELS;
  const FiducialDetector_CPtr fiducialDetector;
  const bool detectFiducials = false;

  m_slamComponents[localSceneID].reset(new SLAMComponent(m_model, localSceneID, imageSource, trackerConfig, mappingMode, trackingMode, fiducialDetector, detectFiducials));
}
