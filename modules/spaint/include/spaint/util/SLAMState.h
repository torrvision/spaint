/**
 * spaint: SLAMState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SLAMSTATE
#define H_SPAINT_SLAMSTATE

#include "ITMImagePtrTypes.h"
#include "ITMObjectPtrTypes.h"
#include "SpaintSurfelScene.h"
#include "SpaintVoxelScene.h"

namespace spaint {

/**
 * \brief An instance of this class represents the state of an RGB-D SLAM reconstruction.
 */
class SLAMState
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The image into which depth input is read each frame. */
  ITMShortImage_Ptr m_inputRawDepthImage;

  /** The image into which RGB input is read each frame. */
  ITMUChar4Image_Ptr m_inputRGBImage;

  /** The surfel render state corresponding to the live camera pose. */
  SurfelRenderState_Ptr m_liveSurfelRenderState;

  /** The voxel render state corresponding to the live camera pose. */
  VoxelRenderState_Ptr m_liveVoxelRenderState;

  /** The current reconstructed surfel scene. */
  SpaintSurfelScene_Ptr m_surfelScene;

  /** The current tracking state (containing the camera pose and additional tracking information used by InfiniTAM). */
  TrackingState_Ptr m_trackingState;

  /** The current view of the scene. */
  View_Ptr m_view;

  /** The current reconstructed voxel scene. */
  SpaintVoxelScene_Ptr m_voxelScene;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the dimensions of the depth images from which the scene is being reconstructed.
   *
   * \return  The dimensions of the depth images from which the scene is being reconstructed.
   */
  const Vector2i& get_depth_image_size() const;

  /**
   * \brief Gets the image into which depth input is read each frame.
   *
   * \return  The image into which depth input is read each frame.
   */
  const ITMShortImage_Ptr& get_input_raw_depth_image();

  /**
   * \brief Gets a copy of the image into which depth input is read each frame.
   *
   * \return  A copy of the image into which depth input is read each frame.
   */
  ITMShortImage_Ptr get_input_raw_depth_image_copy() const;

  /**
   * \brief Gets the image into which RGB input is read each frame.
   *
   * \return  The image into which RGB input is read each frame.
   */
  const ITMUChar4Image_Ptr& get_input_rgb_image();

  /**
   * \brief Gets a copy of the image into which RGB input is read each frame.
   *
   * \return        A copy of the image into which RGB input is read each frame.
   */
  ITMUChar4Image_Ptr get_input_rgb_image_copy() const;

  /**
   * \brief Gets the intrinsic parameters for the camera that is being used to reconstruct the scene.
   *
   * \return  The intrinsic parameters for the camera that is being used to reconstruct the scene.
   */
  const ITMLib::ITMIntrinsics& get_intrinsics() const;

  /**
   * \brief Gets the surfel render state corresponding to the live camera pose for the scene.
   *
   * \return  The surfel render state corresponding to the live camera pose for the scene.
   */
  const SurfelRenderState_Ptr& get_live_surfel_render_state();

  /**
   * \brief Gets the voxel render state corresponding to the live camera pose for the scene.
   *
   * \return  The voxel render state corresponding to the live camera pose for the scene.
   */
  const VoxelRenderState_Ptr& get_live_voxel_render_state();

  /**
   * \brief Gets the current pose of the camera that is being used to reconstruct the scene.
   *
   * \return  The current pose of the camera that is being used to reconstruct the scene.
   */
  const ORUtils::SE3Pose& get_pose() const;

  /**
   * \brief Gets the dimensions of the RGB images from which the scene is being reconstructed.
   *
   * \return  The dimensions of the RGB images from which the scene is being reconstructed.
   */
  const Vector2i& get_rgb_image_size() const;

  /**
   * \brief Gets the surfel scene.
   *
   * \return  The surfel scene.
   */
  const SpaintSurfelScene_Ptr& get_surfel_scene();

  /**
   * \brief Gets the surfel scene.
   *
   * \return  The surfel scene.
   */
  SpaintSurfelScene_CPtr get_surfel_scene() const;

  /**
   * \brief Gets the current tracking state for the scene.
   *
   * \return  The current tracking state for the scene.
   */
  const TrackingState_Ptr& get_tracking_state();

  /**
   * \brief Gets the current tracking state for the scene.
   *
   * \return  The current tracking state for the scene.
   */
  TrackingState_CPtr get_tracking_state() const;

  /**
   * \brief Gets the current view of the scene.
   *
   * \return  The current view of the scene.
   */
  const View_Ptr& get_view();

  /**
   * \brief Gets the current view of the scene.
   *
   * \return  The current view of the scene.
   */
  View_CPtr get_view() const;

  /**
   * \brief Gets the voxel scene.
   *
   * \return  The voxel scene.
   */
  const SpaintVoxelScene_Ptr& get_voxel_scene();

  /**
   * \brief Gets the voxel scene.
   *
   * \return  The voxel scene.
   */
  SpaintVoxelScene_CPtr get_voxel_scene() const;

  /**
   * \brief Sets the image into which depth input is read each frame.
   *
   * \param inputRawDepthImage  The image into which depth input is read each frame.
   */
  void set_input_raw_depth_image(const ITMShortImage_Ptr& inputRawDepthImage);

  /**
   * \brief Sets the image into which RGB input is read each frame.
   *
   * \param inputRGBImage The image into which RGB input is read each frame.
   */
  void set_input_rgb_image(const ITMUChar4Image_Ptr& inputRGBImage);

  /**
   * \brief Sets the surfel render state corresponding to the live camera pose.
   *
   * \param liveSurfelRenderState The surfel render state corresponding to the live camera pose.
   */
  void set_live_surfel_render_state(const SurfelRenderState_Ptr& liveSurfelRenderState);

  /**
   * \brief Sets the voxel render state corresponding to the live camera pose.
   *
   * \param liveRenderState The voxel render state corresponding to the live camera pose.
   */
  void set_live_voxel_render_state(const VoxelRenderState_Ptr& liveVoxelRenderState);

  /**
   * \brief Sets the surfel scene.
   *
   * \param surfelScene The surfel scene.
   */
  void set_surfel_scene(const SpaintSurfelScene_Ptr& surfelScene);

  /**
   * \brief Sets the current tracking state.
   *
   * \param trackingState The new current tracking state.
   */
  void set_tracking_state(const TrackingState_Ptr& trackingState);

  /**
   * \brief Sets the current view of the scene.
   *
   * \param view    The new current view of the scene.
   */
  void set_view(ITMLib::ITMView *view);

  /**
   * \brief Sets the voxel scene.
   *
   * \param voxelScene  The voxel scene.
   */
  void set_voxel_scene(const SpaintVoxelScene_Ptr& voxelScene);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SLAMState> SLAMState_Ptr;
typedef boost::shared_ptr<const SLAMState> SLAMState_CPtr;

}

#endif
