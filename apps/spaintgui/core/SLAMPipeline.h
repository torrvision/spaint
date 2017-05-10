/**
 * spaintgui: SLAMPipeline.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_SLAMPIPELINE
#define H_SPAINTGUI_SLAMPIPELINE

#include "MultiScenePipeline.h"

/**
 * \brief An instance of this class represents a SLAM-only processing pipeline.
 */
class SLAMPipeline : public MultiScenePipeline
{
  //#################### CONSTRUCTORS ####################
public:
  SLAMPipeline(const Settings_Ptr& settings,
               const std::string& resourcesDir,
               const CompositeImageSourceEngine_Ptr& imageSourceEngine,
               const std::string& trackerConfig,
               spaint::SLAMComponent::MappingMode mappingMode = spaint::SLAMComponent::MAP_VOXELS_ONLY,
               spaint::SLAMComponent::TrackingMode trackingMode = spaint::SLAMComponent::TRACK_VOXELS);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void set_mode(Mode mode);
};

#endif
