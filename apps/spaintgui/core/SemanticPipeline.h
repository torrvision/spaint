/**
 * spaintgui: SemanticPipeline.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_SEMANTICPIPELINE
#define H_SPAINTGUI_SEMANTICPIPELINE

#include <spaint/fiducials/FiducialDetector.h>

#include "MultiScenePipeline.h"

/**
 * \brief An instance of this class represents a SemanticPaint processing pipeline.
 */
class SemanticPipeline : public MultiScenePipeline
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a semantic pipeline.
   *
   * \param settings          The settings to use for InfiniTAM.
   * \param resourcesDir      The path to the resources directory.
   * \param maxLabelCount     The maximum number of labels that can be in use.
   * \param imageSourceEngine The engine used to provide input images to the SLAM component for the scene.
   * \param seed              The seed to use for the random number generators used by the voxel samplers.
   * \param trackerType       The type of tracker to use when reconstructing the scene.
   * \param trackerParams     The parameters for the tracker (if any).
   * \param mappingMode       The mapping mode that the scene's SLAM component should use.
   * \param trackingMode      The tracking mode that the scene's SLAM component should use.
   * \param fiducialDetector  The fiducial detector to use (if any).
   * \param detectFiducials   Whether or not to initially detect fiducials in the 3D scene.
   */
  SemanticPipeline(const Settings_Ptr& settings, const std::string& resourcesDir, size_t maxLabelCount,
                   const CompositeImageSourceEngine_Ptr& imageSourceEngine, unsigned int seed,
                   spaint::TrackerType trackerType = spaint::TRACKER_INFINITAM, const std::string& trackerParams = "",
                   spaint::SLAMComponent::MappingMode mappingMode = spaint::SLAMComponent::MAP_VOXELS_ONLY,
                   spaint::SLAMComponent::TrackingMode trackingMode = spaint::SLAMComponent::TRACK_VOXELS,
                   const spaint::FiducialDetector_CPtr& fiducialDetector = spaint::FiducialDetector_CPtr(),
                   bool detectFiducials = false);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void set_mode(Mode mode);
};

#endif
