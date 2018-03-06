/**
 * spaintgui: CollaborativePipeline.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINTGUI_COLLABORATIVEPIPELINE
#define H_SPAINTGUI_COLLABORATIVEPIPELINE

#include <spaint/fiducials/FiducialDetector.h>
#include <spaint/pipelinecomponents/CollaborativeComponent.h>

#include "MultiScenePipeline.h"

/**
 * \brief An instance of this class represents a processing pipeline for collaborative SLAM.
 */
class CollaborativePipeline : public MultiScenePipeline
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** A flag indicating whether or not we should start running collaborative pose estimation. */
  bool m_collaborationStarted;

  /** The collaborative SLAM component used to determine the relative poses between the different agents. */
  spaint::CollaborativeComponent_Ptr m_collaborativeComponent;

  /** Whether or not the user wants fiducials to be detected. */
  bool m_detectFiducials;

  /** The fiducial detector to use (if any). */
  spaint::FiducialDetector_CPtr m_fiducialDetector;

  /** A flag indicating whether or not the "World" scene is being fed from a remote agent. */
  bool m_worldIsRemote;

  //#################### CONSTRUCTORS ####################
public:
  CollaborativePipeline(const Settings_Ptr& settings, const std::string& resourcesDir,
                        const std::vector<CompositeImageSourceEngine_Ptr>& imageSourceEngines,
                        const std::vector<std::string>& trackerConfigs,
                        const std::vector<spaint::SLAMComponent::MappingMode>& mappingModes,
                        const std::vector<spaint::SLAMComponent::TrackingMode>& trackingModes,
                        const spaint::FiducialDetector_CPtr& fiducialDetector = spaint::FiducialDetector_CPtr(),
                        bool detectFiducials = false, const itmx::MappingServer_Ptr& mappingServer = itmx::MappingServer_Ptr(),
                        spaint::CollaborationMode collaborationMode = spaint::CM_LIVE);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual size_t run_main_section();

  /** Override */
  virtual void set_mode(Mode mode);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Adds a SLAM component that will construct a scene using RGB-D frames and poses from a remote client.
   *
   * \param sceneID         The ID to give the scene.
   * \param remoteClientID  The mapping server's ID for the remote client.
   */
  void add_remote_slam_component(const std::string& sceneID, int remoteClientID);

  /**
   * \brief Adds SLAM components for any newly-connected remote clients.
   */
  void check_for_new_clients();
};

#endif
