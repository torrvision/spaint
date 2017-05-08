/**
 * itmx: ICPRefiningRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_ICPREFININGRELOCALISER
#define H_ITMX_ICPREFININGRELOCALISER

#include "RefiningRelocaliser.h"

#include <boost/shared_ptr.hpp>

#include <ITMLib/Core/ITMDenseMapper.h>
#include <ITMLib/Engines/Visualisation/Interface/ITMVisualisationEngine.h>
#include <ITMLib/Objects/Scene/ITMScene.h>

#include "../ITMObjectPtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class allows performing camera relocalisation from RGB-D image pairs followed by an
 *        ICP refinement step.
 *
 * \param VoxelType The type if voxels used to recontruct the scene that will be used during the raycasting step.
 * \param IndexType The type of indexing used to access the reconstructed scene.
 */
template <typename VoxelType, typename IndexType>
class ICPRefiningRelocaliser : public RefiningRelocaliser
{
  //#################### TYPEDEFS ####################
public:
  typedef ITMLib::ITMDenseMapper<VoxelType, IndexType> DenseMapper;
  typedef boost::shared_ptr<DenseMapper> DenseMapper_Ptr;

  typedef ITMLib::ITMScene<VoxelType, IndexType> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

  typedef ITMLib::ITMVisualisationEngine<VoxelType, IndexType> VisualisationEngine;
  typedef boost::shared_ptr<VisualisationEngine> VisualisationEngine_Ptr;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an ICPRefiningRelocaliser.
   *
   * \param relocaliser   A relocaliser whose results will be refined via ICP.
   * \param calibration   The calibration params of tha used to acquire the images to be relocalised.
   * \param imgSize_rgb   The size of the colour images that will be used during the relocalisation.
   * \param imgSize_depth The size of the depth images that will be used during the relocalisation.
   * \param scene         A pointer to the scene that will be reconstructed. Used to raycast the images that will be
   *                      used during ICP.
   * \param settings      A pointer to the InfiniTAM settings used to reconstruct the scene.
   * \param trackerConfig A string specifying the parameters to used to instantiate an ICP tracker used for refinement.
   */
  ICPRefiningRelocaliser(const Relocaliser_Ptr &relocaliser,
                         const ITMLib::ITMRGBDCalib &calibration,
                         const Vector2i imgSize_rgb,
                         const Vector2i imgsize_d,
                         const Scene_Ptr &scene,
                         const Settings_CPtr &settings,
                         const std::string &trackerConfig);

  //#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets a pointer to the refined relocaliser.
   *
   * \return A pointer to the inner relocaliser.
   */
  virtual Relocaliser_Ptr get_inner_relocaliser() const;

  /**
   * \brief Integrates a newly acquired RGB-D image pair into the relocalisation system at a certain pose in the world.
   *
   * \param colourImage     The colour image.
   * \param depthImage      The depth image.
   * \param depthIntrinsics The intrinsic parameters of the depth sensor.
   * \param cameraPose      The position of the camera in the world.
   */
  virtual void integrate_rgbd_pose_pair(const ITMUChar4Image *colourImage,
                                        const ITMFloatImage *depthImage,
                                        const Vector4f &depthIntrinsics,
                                        const ORUtils::SE3Pose &cameraPose);

  /**
   * \brief Attempt to relocalise the location from which an RGB-D image pair is acquired.
   *
   * \param colourImage     The colour image.
   * \param depthImage      The depth image.
   * \param depthIntrinsics The intrinsic parameters of the depth sensor.
   *
   * \return The camera pose if successful, an empty optional otherwise.
   */
  virtual boost::optional<ORUtils::SE3Pose>
      relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f &depthIntrinsics);

  /**
   * \brief Attempt to relocalise the location from which an RGB-D image pair is acquired.
   *        Provides more details on the relcalisation phase.
   *
   * \param colourImage       The colour image.
   * \param depthImage        The depth image.
   * \param depthIntrinsics   The intrinsic parameters of the depth sensor.
   * \param refinementDetails A details structure that will be filled with informations captured during the
   *                          relocalisation.
   *
   * \return The camera pose if successful, an empty optional otherwise.
   */
  virtual boost::optional<ORUtils::SE3Pose> relocalise(const ITMUChar4Image *colourImage,
                                                       const ITMFloatImage *depthImage,
                                                       const Vector4f &depthIntrinsics,
                                                       RefinementDetails &refinementDetails);

  /**
   * \brief Resets the relocaliser allowing the integration of informations on a new area.
   */
  virtual void reset();

  /**
   * \brief Updates the contents of the relocaliser when spare processing time is available. Can perform bookkeeping
   *        operations.
   */
  virtual void update();

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** A DenseMapper used to find visible blocks in the scene. */
  DenseMapper_Ptr m_denseMapper;

  /** A low level engine used by the tracker. */
  LowLevelEngine_Ptr m_lowLevelEngine;

  /** The wrapped relocaliser. */
  Relocaliser_Ptr m_relocaliser;

  /** The reconstructed scene. */
  Scene_Ptr m_scene;

  /** Settings used when reconstructing the scene. */
  Settings_CPtr m_settings;

  /** A tracker used to refine the relocalised poses. */
  Tracker_Ptr m_tracker;

  /** A tracking controller used to setup and perform the actual refinement. */
  TrackingController_Ptr m_trackingController;

  /** A tracking state used to hold refinement results. */
  TrackingState_Ptr m_trackingState;

  /** A visualization engine used to perform the raycasting. */
  VisualisationEngine_Ptr m_visualisationEngine;

  /** A view used to pass the input images to the tracker and the visualization engine. */
  View_Ptr m_view;

  /** A renderState used to hold the raycasting results. */
  VoxelRenderState_Ptr m_voxelRenderState;
};

} // namespace itmx

#endif // H_ITMX_ICPREFININGRELOCALISER
