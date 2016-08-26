/**
 * spaint: SLAMComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SLAMCOMPONENT
#define H_SPAINT_SLAMCOMPONENT

#include <InputSource/CompositeImageSourceEngine.h>

#include <ITMLib/Core/ITMDenseMapper.h>
#include <ITMLib/Core/ITMDenseSurfelMapper.h>
#include <ITMLib/Core/ITMTrackingController.h>
#include <ITMLib/Engines/LowLevel/Interface/ITMLowLevelEngine.h>
#include <ITMLib/Engines/ViewBuilding/Interface/ITMViewBuilder.h>
#include <ITMLib/Objects/Misc/ITMIMUCalibrator.h>

#include <RelocLib/PoseDatabase.h>
#include <RelocLib/Relocaliser.h>

#include "SLAMContext.h"
#include "../trackers/FallibleTracker.h"
#include "../trackers/TrackerType.h"

namespace spaint {

/**
 * \brief An instance of this pipeline component can be used to perform simultaneous localisation and mapping (SLAM).
 */
class SLAMComponent
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<InputSource::CompositeImageSourceEngine> CompositeImageSourceEngine_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMDenseMapper<SpaintVoxel,ITMVoxelIndex> > DenseMapper_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMDenseSurfelMapper<SpaintSurfel> > DenseSurfelMapper_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMIMUCalibrator> IMUCalibrator_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMLowLevelEngine> LowLevelEngine_Ptr;
  typedef boost::shared_ptr<RelocLib::PoseDatabase> PoseDatabase_Ptr;
  typedef boost::shared_ptr<RelocLib::Relocaliser> Relocaliser_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMTrackingController> TrackingController_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMTrackingController> TrackingController_CPtr;
  typedef boost::shared_ptr<ITMLib::ITMViewBuilder> ViewBuilder_Ptr;

  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief The values of this enumeration denote the different mapping modes that can be used by a SLAM component.
   */
  enum MappingMode
  {
    /** Produce both voxel and surfel maps. */
    MAP_BOTH,

    /** Produce only a voxel map. */
    MAP_VOXELS_ONLY
  };

  /**
   * \brief The values of this enumeration denote the different tracking modes that can be used by a SLAM component.
   */
  enum TrackingMode
  {
    /** Track against the surfel map. */
    TRACK_SURFELS,

    /** Track against the voxel map. */
    TRACK_VOXELS
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The shared context needed for SLAM. */
  SLAMContext_Ptr m_context;

  /** The dense surfel mapper. */
  DenseSurfelMapper_Ptr m_denseSurfelMapper;

  /** The dense voxel mapper. */
  DenseMapper_Ptr m_denseVoxelMapper;

  /** A pointer to a tracker that can detect tracking failures (if available). */
  FallibleTracker *m_fallibleTracker;

  /** The number of frames for which fusion has been run. */
  size_t m_fusedFramesCount;

  /** Whether or not the user wants fusion to be run. */
  bool m_fusionEnabled;

  /** The engine used to provide input images to the fusion process. */
  CompositeImageSourceEngine_Ptr m_imageSourceEngine;

  /** The IMU calibrator. */
  IMUCalibrator_Ptr m_imuCalibrator;

  /**
   * A number of initial frames to fuse, regardless of their tracking quality.
   * Tracking quality can be poor in the first few frames, when there is only
   * a limited model against which to track. By forcibly fusing these frames,
   * we prevent poor tracking quality from stopping the reconstruction. After
   * these frames have been fused, only frames with a good tracking result will
   * be fused.
   */
  size_t m_initialFramesToFuse;

  /** The remaining number of frames for which we need to achieve good tracking before we can add another keyframe. */
  size_t m_keyframeDelay;

  /** The engine used to perform low-level image processing operations. */
  LowLevelEngine_Ptr m_lowLevelEngine;

  /** The mapping mode to use. */
  MappingMode m_mappingMode;

  /** The database of previous poses for relocalisation. */
  PoseDatabase_Ptr m_poseDatabase;

  /** The relocaliser. */
  Relocaliser_Ptr m_relocaliser;

  /** The ID of the scene to reconstruct. */
  std::string m_sceneID;

  /** The tracker. */
  Tracker_Ptr m_tracker;

  /**
   * The parameters for the tracker (if any). For example, this would be the host on which the
   * Vicon software is running (e.g. "<IP address>:<port>") if we're using the Vicon tracker.
   */
  std::string m_trackerParams;

  /** The type of tracker to use. */
  TrackerType m_trackerType;

  /** The tracking controller. */
  TrackingController_Ptr m_trackingController;

  /** The tracking mode to use. */
  TrackingMode m_trackingMode;

  /** The view builder. */
  ViewBuilder_Ptr m_viewBuilder;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a SLAM component.
   *
   * \param context           The shared context needed for SLAM.
   * \param sceneID           The ID of the scene to reconstruct.
   * \param imageSourceEngine The engine used to provide input images to the fusion process.
   * \param trackerType       The type of tracker to use.
   * \param trackerParams     The parameters for the tracker (if any).
   * \param mappingMode       The mapping mode to use.
   * \param trackingMode      The tracking mode to use.
   */
  SLAMComponent(const SLAMContext_Ptr& context, const std::string& sceneID, const CompositeImageSourceEngine_Ptr& imageSourceEngine,
                TrackerType trackerType, const std::string& trackerParams, MappingMode mappingMode = MAP_VOXELS_ONLY,
                TrackingMode trackingMode = TRACK_VOXELS);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets whether or not the user wants fusion to be run.
   *
   * \return  true, if the user wants fusion to be run, or false otherwise.
   */
  bool get_fusion_enabled() const;

  /**
   * \brief Attempts to run the SLAM component for a single frame.
   *
   * \return  true, if a frame was processed, or false otherwise.
   */
  bool run();

  /**
   * \brief Sets whether or not the user wants fusion to be run.
   *
   * Note: Just because the user wants fusion to be run doesn't mean that it necessarily will be on every frame.
   *       In particular, we prevent fusion when we know we have lost tracking, regardless of this setting.
   *
   * \param fusionEnabled Whether or not the user wants fusion to be run.
   */
  void set_fusion_enabled(bool fusionEnabled);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes a hybrid tracker that refines the results of a primary tracker using ICP.
   *
   * \param primaryTracker  The primary tracker (e.g. a Rift or Vicon tracker).
   * \return                The hybrid tracker.
   */
  ITMLib::ITMTracker *make_hybrid_tracker(ITMLib::ITMTracker *primaryTracker) const;

  /**
   * \brief Render from the live camera position to prepare for tracking.
   *
   * \param trackingMode  The tracking mode to use.
   */
  void prepare_for_tracking(TrackingMode trackingMode);

  /**
   * \brief Sets up the tracker.
   */
  void setup_tracker();
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SLAMComponent> SLAMComponent_Ptr;

}

#endif
