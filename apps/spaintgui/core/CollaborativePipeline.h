/**
 * spaintgui: CollaborativePipeline.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINTGUI_COLLABORATIVEPIPELINE
#define H_SPAINTGUI_COLLABORATIVEPIPELINE

#include <spaint/fiducials/FiducialDetector.h>

#include "MultiScenePipeline.h"

/**
 * \brief An instance of this class represents a processing pipeline for collaborative SLAM.
 */
class CollaborativePipeline : public MultiScenePipeline
{
  //#################### CONSTRUCTORS ####################
public:
  CollaborativePipeline(const Settings_Ptr& settings, const std::string& resourcesDir,
                        const std::vector<CompositeImageSourceEngine_Ptr>& imageSourceEngines,
                        const std::vector<std::string>& trackerConfigs,
                        const std::vector<spaint::SLAMComponent::MappingMode>& mappingModes,
                        const std::vector<spaint::SLAMComponent::TrackingMode>& trackingModes,
                        const spaint::FiducialDetector_CPtr& fiducialDetector = spaint::FiducialDetector_CPtr(),
                        bool detectFiducials = false);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void set_mode(Mode mode);
};

#endif
