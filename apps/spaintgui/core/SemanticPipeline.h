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
   * \param trackerConfig     The tracker configuration to use when reconstructing the scene.
   * \param mappingMode       The mapping mode that the scene's SLAM component should use.
   * \param trackingMode      The tracking mode that the scene's SLAM component should use.
   * \param modelSpecifier    An optional model specifier denoting a location from which to load pre-reconstructed voxel and surfel models.
   * \param fiducialDetector  The fiducial detector to use (if any).
   * \param detectFiducials   Whether or not to initially detect fiducials in the 3D scene.
   * \param mappingServer     The remote mapping server (if any).
   */
  SemanticPipeline(const Settings_Ptr& settings, const std::string& resourcesDir, size_t maxLabelCount,
                   const CompositeImageSourceEngine_Ptr& imageSourceEngine, unsigned int seed, const std::string& trackerConfig,
                   spaint::SLAMComponent::MappingMode mappingMode = spaint::SLAMComponent::MAP_VOXELS_ONLY,
                   spaint::SLAMComponent::TrackingMode trackingMode = spaint::SLAMComponent::TRACK_VOXELS,
                   const boost::optional<boost::filesystem::path>& modelDir = boost::none,
                   const spaint::FiducialDetector_CPtr& fiducialDetector = spaint::FiducialDetector_CPtr(),
                   bool detectFiducials = false, const itmx::MappingServer_Ptr& mappingServer = itmx::MappingServer_Ptr());

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void set_mode(Mode mode);
};

#endif
