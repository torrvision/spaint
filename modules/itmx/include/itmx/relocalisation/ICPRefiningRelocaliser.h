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
   */
  ICPRefiningRelocaliser(const Relocaliser_Ptr &relocaliser,
                         const Scene_Ptr &scene,
                         const ITMLib::ITMLibSettings &settings,
                         const ITMLib::ITMRGBDCalib &calibration,
                         const Vector2i imgSize_rgb,
                         const Vector2i imgsize_d,
                         const std::string &trackerConfig);

  //#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################
public:
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
  DenseMapper_Ptr m_denseMapper;

  LowLevelEngine_Ptr m_lowLevelEngine;

  Relocaliser_Ptr m_relocaliser;

  Scene_Ptr m_scene;

  ITMLib::ITMLibSettings m_itmLibSettings;

  Tracker_Ptr m_tracker;

  TrackingController_Ptr m_trackingController;

  TrackingState_Ptr m_trackingState;

  VisualisationEngine_Ptr m_visualisationEngine;

  View_Ptr m_view;

  VoxelRenderState_Ptr m_voxelRenderState;
};

} // namespace itmx

#endif // H_ITMX_ICPREFININGRELOCALISER
