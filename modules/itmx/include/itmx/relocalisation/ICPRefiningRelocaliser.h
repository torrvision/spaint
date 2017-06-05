/**
 * itmx: ICPRefiningRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_ICPREFININGRELOCALISER
#define H_ITMX_ICPREFININGRELOCALISER

#include <ITMLib/Core/ITMDenseMapper.h>
#include <ITMLib/Engines/Visualisation/Interface/ITMVisualisationEngine.h>
#include <ITMLib/Objects/Scene/ITMScene.h>

#include <tvgutil/filesystem/SequentialPathGenerator.h>
#include <tvgutil/timing/AverageTimer.h>

#include "../base/ITMObjectPtrTypes.h"
#include "RefiningRelocaliser.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to refine the results of another relocaliser using ICP.
 *
 * \tparam VoxelType  The type of voxel used to recontruct the scene that will be used during the raycasting step.
 * \tparam IndexType  The type of indexing used to access the reconstructed scene.
 */
template <typename VoxelType, typename IndexType>
class ICPRefiningRelocaliser : public RefiningRelocaliser
{
  //#################### TYPEDEFS ####################
private:
  typedef tvgutil::AverageTimer<boost::chrono::microseconds> AverageTimer;
  typedef ITMLib::ITMDenseMapper<VoxelType,IndexType> DenseMapper;
  typedef boost::shared_ptr<DenseMapper> DenseMapper_Ptr;
  typedef ITMLib::ITMScene<VoxelType,IndexType> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;
  typedef ITMLib::ITMVisualisationEngine<VoxelType,IndexType> VisualisationEngine;
  typedef boost::shared_ptr<const VisualisationEngine> VisualisationEngine_CPtr;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The dense mapper used to find visible blocks in the voxel scene. */
  DenseMapper_Ptr m_denseVoxelMapper;

  /** The low-level engine used by the tracker. */
  LowLevelEngine_CPtr m_lowLevelEngine;

  /** The path generator used when saving the relocalised poses. */
  mutable boost::optional<tvgutil::SequentialPathGenerator> m_posePathGenerator;

  /** Whether or not to save the relocalised poses. */
  bool m_savePoses;

  /** The scene being viewed from the camera. */
  Scene_Ptr m_scene;

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_settings;

  /** The timer used to profile the relocalisation calls. */
  mutable AverageTimer m_timerRelocalisation;

  /** The timer used to profile the training calls. */
  AverageTimer m_timerTraining;

  /** The timer used to profile the update calls. */
  AverageTimer m_timerUpdate;

  /** Whether or not timers are enabled and stats are printed on destruction. */
  bool m_timersEnabled;

  /** The ICP tracker used to refine the relocalised poses. */
  Tracker_Ptr m_tracker;

  /** The tracking controller used to set up and perform the actual refinement. */
  TrackingController_Ptr m_trackingController;

  /** The tracking state used to hold the refinement results. */
  TrackingState_Ptr m_trackingState;

  /** The visualisation engine used to perform the raycasting. */
  VisualisationEngine_CPtr m_visualisationEngine;

  /** The current view of the scene. */
  View_Ptr m_view;

  /** The voxel render state used to hold the raycasting results. */
  mutable VoxelRenderState_Ptr m_voxelRenderState;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an ICP-based refining relocaliser.
   *
   * \param innerRelocaliser    The relocaliser whose results are being refined using ICP.
   * \param trackerConfig       A configuration string used to specify the parameters of the ICP tracker.
   * \param rgbImageSize        The size of the colour images produced by the camera.
   * \param depthImageSize      The size of the depth images produced by the camera.
   * \param calib               The calibration parameters of the camera whose pose is to be estimated.
   * \param scene               The scene being viewed from the camera.
   * \param settings            The settings to use for InfiniTAM.
   * \param lowLevelEngine      The low-level engine used by the tracker.
   * \param visualisationEngine The visualisation engine used to perform the raycasting.
   */
  ICPRefiningRelocaliser(const Relocaliser_Ptr& innerRelocaliser, const std::string& trackerConfig,
                         const Vector2i& rgbImageSize, const Vector2i& depthImageSize,
                         const ITMLib::ITMRGBDCalib& calib, const Scene_Ptr& scene,
                         const Settings_CPtr& settings, const LowLevelEngine_CPtr& lowLevelEngine,
                         const VisualisationEngine_CPtr& visualisationEngine);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the relocaliser.
   */
  ~ICPRefiningRelocaliser();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  ICPRefiningRelocaliser(const ICPRefiningRelocaliser&);
  ICPRefiningRelocaliser& operator=(const ICPRefiningRelocaliser&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual boost::optional<Result> relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage,
                                             const Vector4f& depthIntrinsics) const;

  /** Override */
  virtual boost::optional<Result> relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage,
                                             const Vector4f& depthIntrinsics, boost::optional<ORUtils::SE3Pose>& initialPose) const;

  /** Override */
  virtual void reset();

  /** Override */
  virtual void train(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage,
                     const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose);

  /** Override */
  virtual void update();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Saves the relocalised and refined poses in text files so that they can be used later (e.g. for evaluation).
   *
   * \note Saving happens only if m_savePoses is true.
   *
   * \param relocalisedPose The relocalised pose before refinement.
   * \param refinedPose     The result of refining the relocalised pose.
   */
  void save_poses(const Matrix4f& relocalisedPose, const Matrix4f& refinedPose) const;

  /**
   * \brief Starts the specified timer (waiting for all CUDA operations to terminate first, if necessary).
   *
   * \param timer The timer to start.
   */
  void start_timer(AverageTimer& timer) const;

  /**
   * \brief Stops the specified timer (waiting for all CUDA operations to terminate first, if necessary).
   *
   * \param timer The timer to stop.
   */
  void stop_timer(AverageTimer& timer) const;
};

}

#endif
