/**
 * spaint: SpaintModel
 */

#ifndef H_SPAINT_SPAINTMODEL
#define H_SPAINT_SPAINTMODEL

#include <boost/shared_ptr.hpp>

#include <Engine/ImageSourceEngine.h>

#include "util/SpaintVoxel.h"

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
private:
  typedef boost::shared_ptr<InfiniTAM::Engine::ImageSourceEngine> ImageSourceEngine_Ptr;
  typedef ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;
  typedef boost::shared_ptr<ITMTrackingState> TrackingState_Ptr;
  typedef boost::shared_ptr<const ITMTrackingState> TrackingState_CPtr;
  typedef boost::shared_ptr<ITMView> View_Ptr;
public:
  typedef boost::shared_ptr<const Scene> Scene_CPtr;
  typedef boost::shared_ptr<const ITMView> View_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The current reconstructed scene. */
  Scene_Ptr m_scene;

  /** The settings to use for InfiniTAM. */
  ITMLibSettings m_settings;

  /** The current tracking state (containing the camera pose and additional tracking information used by InfiniTAM). */
  TrackingState_Ptr m_trackingState;

  /** The current view of the scene. */
  View_Ptr m_view;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a model.
   *
   * \param settings          The settings to use for InfiniTAM.
   * \param imageSourceEngine The InfiniTAM engine used to provide input images to the fusion pipeline.
   */
  SpaintModel(const ITMLibSettings& settings, const ImageSourceEngine_Ptr& imageSourceEngine);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the intrinsic parameters for the camera that is being used to reconstruct the scene.
   *
   * \return  The intrinsic parameters for the camera.
   */
  const ITMIntrinsics& get_intrinsics() const;

  /**
   * \brief Gets the current pose of the camera that is being used to reconstruct the scene.
   *
   * \return  The current camera pose.
   */
  const ITMPose& get_pose() const;

  /**
   * \brief Gets the current reconstructed scene.
   *
   * \return  The current reconstructed scene.
   */
  Scene_CPtr get_scene() const;

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
  View_CPtr get_view() const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SpaintModel> SpaintModel_Ptr;
typedef boost::shared_ptr<const SpaintModel> SpaintModel_CPtr;

}

#endif
