/**
 * spaint: SLAMContext.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SLAMCONTEXT
#define H_SPAINT_SLAMCONTEXT

#include <map>

#include <ITMLib/Engines/Visualisation/Interface/ITMSurfelVisualisationEngine.h>
#include <ITMLib/Engines/Visualisation/Interface/ITMVisualisationEngine.h>
#include <ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include "../util/ITMImagePtrTypes.h"
#include "../util/ITMObjectPtrTypes.h"
#include "../util/SpaintSurfelScene.h"
#include "../util/SpaintVoxelScene.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one provides the shared context needed by SLAM components.
 */
class SLAMContext
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMSurfelVisualisationEngine<SpaintSurfel> > SurfelVisualisationEngine_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMVisualisationEngine<SpaintVoxel,ITMVoxelIndex> > VoxelVisualisationEngine_CPtr;

  //#################### NESTED TYPES ####################
private:
  /**
   * \brief An instance of this struct contains the shared SLAM context for a single reconstructed scene.
   */
  struct SingleSceneContext
  {
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
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The shared SLAM contexts for the various reconstructed scenes. */
  std::map<std::string,SingleSceneContext> m_singleSceneContexts;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the SLAM context.
   */
  virtual ~SLAMContext() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  virtual const Settings_CPtr& get_settings() const = 0;
  virtual SurfelVisualisationEngine_CPtr get_surfel_visualisation_engine() const = 0;
  virtual VoxelVisualisationEngine_CPtr get_voxel_visualisation_engine() const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the dimensions of the depth images from which the specified scene is being reconstructed.
   *
   * \param sceneID The scene ID.
   * \return        The dimensions of the depth images from which the specified scene is being reconstructed.
   */
  virtual const Vector2i& get_depth_image_size(const std::string& sceneID) const;

  /**
   * \brief Gets the image into which depth input is read each frame for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The image into which depth input is read each frame for the specified scene.
   */
  virtual const ITMShortImage_Ptr& get_input_raw_depth_image(const std::string& sceneID);

  /**
   * \brief Gets the image into which RGB input is read each frame for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The image into which RGB input is read each frame for the specified scene.
   */
  virtual const ITMUChar4Image_Ptr& get_input_rgb_image(const std::string& sceneID);

  /**
   * \brief Gets the intrinsic parameters for the camera that is being used to reconstruct the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The intrinsic parameters for the camera that is being used to reconstruct the specified scene.
   */
  virtual const ITMLib::ITMIntrinsics& get_intrinsics(const std::string& sceneID) const;

  /**
   * \brief Gets the surfel render state corresponding to the live camera pose for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The surfel render state corresponding to the live camera pose for the specified scene.
   */
  virtual const SurfelRenderState_Ptr& get_live_surfel_render_state(const std::string& sceneID);

  /**
   * \brief Gets the voxel render state corresponding to the live camera pose for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The voxel render state corresponding to the live camera pose for the specified scene.
   */
  virtual const VoxelRenderState_Ptr& get_live_voxel_render_state(const std::string& sceneID);

  /**
   * \brief Gets the current pose of the camera that is being used to reconstruct the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The current pose of the camera that is being used to reconstruct the specified scene.
   */
  virtual const ORUtils::SE3Pose& get_pose(const std::string& sceneID) const;

  /**
   * \brief Gets the dimensions of the RGB images from which the specified scene is being reconstructed.
   *
   * \param sceneID The scene ID.
   * \return        The dimensions of the RGB images from which the specified scene is being reconstructed.
   */
  virtual const Vector2i& get_rgb_image_size(const std::string& sceneID) const;

  /**
   * \brief Gets the specified surfel scene.
   *
   * \param sceneID The scene ID.
   * \return        The corresponding scene.
   */
  virtual const SpaintSurfelScene_Ptr& get_surfel_scene(const std::string& sceneID);

  /**
   * \brief Gets the specified surfel scene.
   *
   * \param sceneID The scene ID.
   * \return        The corresponding scene.
   */
  virtual SpaintSurfelScene_CPtr get_surfel_scene(const std::string& sceneID) const;

  /**
   * \brief Gets the current tracking state for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The current tracking state for the specified scene.
   */
  virtual const TrackingState_Ptr& get_tracking_state(const std::string& sceneID);

  /**
   * \brief Gets the current tracking state for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The current tracking state for the specified scene.
   */
  virtual TrackingState_CPtr get_tracking_state(const std::string& sceneID) const;

  /**
   * \brief Gets the current view of the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The current view of the specified scene.
   */
  virtual const View_Ptr& get_view(const std::string& sceneID);

  /**
   * \brief Gets the current view of the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The current view of the specified scene.
   */
  virtual View_CPtr get_view(const std::string& sceneID) const;

  /**
   * \brief Gets the specified voxel scene.
   *
   * \param sceneID The scene ID.
   * \return        The corresponding scene.
   */
  virtual const SpaintVoxelScene_Ptr& get_voxel_scene(const std::string& sceneID);

  /**
   * \brief Gets the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The corresponding scene.
   */
  virtual SpaintVoxelScene_CPtr get_voxel_scene(const std::string& sceneID) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Gets the shared SLAM context for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The shared SLAM context for the specified scene.
   */
  const SingleSceneContext& get_single_scene_context(const std::string& sceneID) const;

  /**
   * \brief Sets the target of the specified shared pointer to that of the specified raw pointer.
   *
   * If the two pointers already point to the same place, this is a no-op.
   *
   * \param dest  The shared pointer whose target is to be set.
   * \param src   The raw pointer from which to set it.
   */
  template <typename T>
  void set_if_different(boost::shared_ptr<T>& dest, T *src)
  {
    if(dest.get() != src) dest.reset(src);
  }

  /**
   * \brief Sets the image into which depth input is read each frame for the specified scene.
   *
   * \param sceneID             The scene ID.
   * \param inputRawDepthImage  The image into which depth input is read each frame for the specified scene.
   */
  void set_input_raw_depth_image(const std::string& sceneID, ITMShortImage *inputRawDepthImage);

  /**
   * \brief Sets the image into which RGB input is read each frame for the specified scene.
   *
   * \param sceneID       The scene ID.
   * \param inputRGBImage The image into which RGB input is read each frame.
   */
  void set_input_rgb_image(const std::string& sceneID, ITMUChar4Image *inputRGBImage);

  /**
   * \brief Sets the surfel render state corresponding to the live camera pose for the specified scene.
   *
   * \param sceneID               The scene ID.
   * \param liveSurfelRenderState The surfel render state corresponding to the live camera pose for the specified scene.
   */
  void set_live_surfel_render_state(const std::string& sceneID, ITMLib::ITMSurfelRenderState *liveSurfelRenderState);

  /**
   * \brief Sets the voxel render state corresponding to the live camera pose for the specified scene.
   *
   * \param sceneID         The scene ID.
   * \param liveRenderState The voxel render state corresponding to the live camera pose for the specified scene.
   */
  void set_live_voxel_render_state(const std::string& sceneID, ITMLib::ITMRenderState *liveVoxelRenderState);

  /**
   * \brief Sets the specified surfel scene.
   *
   * \param sceneID     The scene ID.
   * \param surfelScene The surfel scene.
   */
  void set_surfel_scene(const std::string& sceneID, SpaintSurfelScene *surfelScene);

  /**
   * \brief Sets the current tracking state for the specified scene.
   *
   * \param sceneID       The scene ID.
   * \param trackingState The new current tracking state for the specified scene.
   */
  void set_tracking_state(const std::string& sceneID, ITMLib::ITMTrackingState *trackingState);

  /**
   * \brief Sets the current view of the specified scene.
   *
   * \param sceneID The scene ID.
   * \param view    The new current view of the specified scene.
   */
  void set_view(const std::string& sceneID, ITMLib::ITMView *view);

  /**
   * \brief Sets the specified voxel scene.
   *
   * \param sceneID     The scene ID.
   * \param voxelScene  The voxel scene.
   */
  void set_voxel_scene(const std::string& sceneID, SpaintVoxelScene *voxelScene);

  //#################### FRIENDS ####################

  friend class SLAMComponent;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SLAMContext> SLAMContext_Ptr;

}

#endif
