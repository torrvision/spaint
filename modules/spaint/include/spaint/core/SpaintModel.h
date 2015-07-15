/**
 * spaint: SpaintModel
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_SPAINTMODEL
#define H_SPAINT_SPAINTMODEL

#include <boost/shared_ptr.hpp>

#include <Engine/ImageSourceEngine.h>

#include "../util/LabelManager.h"

namespace spaint {

/**
 * \brief An instance of this class represents our model of the spaint scenario.
 *
 * The scenario we model is one of reconstructing a scene from a series of RGB-D images with known (tracked) pose,
 * and labelling it interactively using various user input modalities.
 */
class SpaintModel
{
  //#################### TYPEDEFS ####################
public:
  typedef ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;
  typedef boost::shared_ptr<const Scene> Scene_CPtr;
  typedef boost::shared_ptr<const ITMLibSettings> Settings_CPtr;
  typedef boost::shared_ptr<ITMTrackingState> TrackingState_Ptr;
  typedef boost::shared_ptr<const ITMTrackingState> TrackingState_CPtr;
  typedef boost::shared_ptr<ITMView> View_Ptr;
  typedef boost::shared_ptr<const ITMView> View_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The dimensions of the depth images from which the scene is being reconstructed. */
  Vector2i m_depthImageSize;

  /** The label manager. */
  LabelManager_Ptr m_labelManager;

  /** The path tot he resources directory. */
  std::string m_resourcesDir;

  /** The dimensions of the RGB images from which the scene is being reconstructed. */
  Vector2i m_rgbImageSize;

  /** The current reconstructed scene. */
  Scene_Ptr m_scene;

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_settings;

  /** The current tracking state (containing the camera pose and additional tracking information used by InfiniTAM). */
  TrackingState_Ptr m_trackingState;

  /** The current view of the scene. */
  View_Ptr m_view;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a model.
   *
   * \param scene           The InfiniTAM scene.
   * \param rgbImageSize    The dimensions of the RGB images from which the scene is being reconstructed.
   * \param depthImageSize  The dimensions of the depth images from which the scene is being reconstructed.
   * \param trackingState   The current tracking state (containing the camera pose and additional tracking information used by InfiniTAM).
   * \param settings        The settings to use for InfiniTAM.
   * \param resourcesDir    The path to the resources directory.
   */
  SpaintModel(const Scene_Ptr& scene, const Vector2i& rgbImageSize, const Vector2i& depthImageSize, const TrackingState_Ptr& trackingState,
              const Settings_CPtr& settings, const std::string& resourcesDir);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the dimensions of the depth images from which the scene is being reconstructed.
   *
   * \return  The dimensions of the depth images from which the scene is being reconstructed.
   */
  const Vector2i& get_depth_image_size() const;

  /**
   * \brief Gets the intrinsic parameters for the camera that is being used to reconstruct the scene.
   *
   * \return  The intrinsic parameters for the camera.
   */
  const ITMIntrinsics& get_intrinsics() const;

  /**
   * \brief Gets the label manager.
   *
   * \return  The label manager.
   */
  const LabelManager_Ptr& get_label_manager();

  /**
   * \brief Gets the label manager.
   *
   * \return  The label manager.
   */
  LabelManager_CPtr get_label_manager() const;

  /**
   * \brief Gets the current pose of the camera that is being used to reconstruct the scene.
   *
   * \return  The current camera pose.
   */
  const ITMPose& get_pose() const;

  /**
   * \brief Gets the path to the resources directory.
   *
   * \return The path to the resources directory.
   */
  std::string get_resources_dir() const;

  /**
   * \brief Gets the dimensions of the RGB images from which the scene is being reconstructed.
   *
   * \return  The dimensions of the RGB images from which the scene is being reconstructed.
   */
  const Vector2i& get_rgb_image_size() const;

  /**
   * \brief Gets the current reconstructed scene.
   *
   * \return  The current reconstructed scene.
   */
  const Scene_Ptr& get_scene();

  /**
   * \brief Gets the current reconstructed scene.
   *
   * \return  The current reconstructed scene.
   */
  Scene_CPtr get_scene() const;

  /**
   * \brief Gets the settings to use for InfiniTAM.
   *
   * \return  The settings to use for InfiniTAM.
   */
  const Settings_CPtr& get_settings() const;

  /**
   * \brief Gets the current tracking state.
   *
   * \return  The current tracking state.
   */
  const TrackingState_Ptr& get_tracking_state();

  /**
   * \brief Gets the current tracking state.
   *
   * \return  The current tracking state.
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
   * \brief Sets the current view of the scene.
   *
   * \param view  The new current view of the scene.
   */
  void set_view(ITMView *view);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SpaintModel> SpaintModel_Ptr;
typedef boost::shared_ptr<const SpaintModel> SpaintModel_CPtr;

}

#endif
