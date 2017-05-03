/**
 * itmx: ICPRefiningRelocaliser.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ICPRefiningRelocaliser.h"

#include <ITMLib/Core/ITMTrackingController.h>
#include <ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
#include <ITMLib/Trackers/ITMTrackerFactory.h>

namespace itmx {

//#################### CONSTRUCTORS ####################

template <typename VoxelType, typename IndexType>
ICPRefiningRelocaliser<VoxelType, IndexType>::ICPRefiningRelocaliser(const Relocaliser_Ptr &relocaliser,
                                                                     const Scene_Ptr &scene,
                                                                     const ITMLib::ITMLibSettings &settings,
                                                                     const ITMLib::ITMRGBDCalib &calibration,
                                                                     const Vector2i imgSize_rgb,
                                                                     const Vector2i imgsize_d,
                                                                     const std::string &trackerConfig)
  : m_relocaliser(relocaliser), m_scene(scene), m_itmLibSettings(settings)
{
  m_denseMapper.reset(new DenseMapper(m_itmLibSettings.deviceType));

  m_lowLevelEngine.reset(ITMLib::ITMLowLevelEngineFactory::MakeLowLevelEngine(settings.deviceType));

  m_tracker.reset(ITMLib::ITMTrackerFactory::Instance().Make(settings.deviceType,
                                                             trackerConfig.c_str(),
                                                             imgSize_rgb,
                                                             imgsize_d,
                                                             m_lowLevelEngine,
                                                             NULL,
                                                             m_itmLibSettings.sceneParams));

  m_trackingController.reset(new ITMLib::ITMTrackingController(m_tracker.get(), &m_itmLibSettings));

  m_trackingState.reset(new ITMLib::ITMTrackingState(imgsize_d, m_itmLibSettings.GetMemoryType()));

  m_visualisationEngine.reset(ITMLib::ITMVisualisationEngineFactory::MakeVisualisationEngine<VoxelType, IndexType>(
      m_itmLibSettings.deviceType));

  m_view.reset(new ITMLib::ITMView(
      calibration, imgSize_rgb, imgsize_d, m_itmLibSettings.deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA));
  // Manually delete the rgb and depth members to prevent leaks, since we will be overwriting them in each call to the
  // relocaliser (!!!).
  delete m_view->rgb;
  m_view->rgb = NULL;
  delete m_view->depth;
  m_view->depth = NULL;

  m_voxelRenderState.reset(new ITMLib::ITMRenderState(imgsize_d,
                                                      m_itmLibSettings.sceneParams.viewFrustum_min,
                                                      m_itmLibSettings.sceneParams.viewFrustum_max,
                                                      m_itmLibSettings.GetMemoryType()));
}

//#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType, IndexType>::integrate_rgbd_pose_pair(const ITMUChar4Image *colourImage,
                                                                            const ITMFloatImage *depthImage,
                                                                            const Vector4f &depthIntrinsics,
                                                                            const ORUtils::SE3Pose &cameraPose)
{
  m_relocaliser->integrate_rgbd_pose_pair(colourImage, depthImage, depthIntrinsics, cameraPose);
}

} // namespace itmx
