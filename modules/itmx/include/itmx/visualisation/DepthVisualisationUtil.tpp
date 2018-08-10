/**
 * itmx: DepthVisualisationUtil.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "DepthVisualisationUtil.h"

#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>

#include <orx/geometry/GeometryUtil.h>

#include "../util/CameraPoseConverter.h"

namespace itmx {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

template <typename VoxelType, typename IndexType>
void DepthVisualisationUtil<VoxelType,IndexType>::generate_depth_from_voxels(const ORFloatImage_Ptr& output, const Scene_CPtr& scene, const ORUtils::SE3Pose& pose,
                                                                             const ITMLib::ITMIntrinsics& intrinsics, VoxelRenderState_Ptr& renderState,
                                                                             DepthVisualiser::DepthType depthType, const VoxelVisualisationEngine_CPtr& voxelVisualisationEngine,
                                                                             const itmx::DepthVisualiser_CPtr& depthVisualiser, const Settings_CPtr& settings)
{
  if(!scene)
  {
    output->Clear();
    return;
  }

  if(!renderState) renderState.reset(ITMRenderStateFactory<IndexType>::CreateRenderState(output->noDims, scene->sceneParams, settings->GetMemoryType()));

  voxelVisualisationEngine->FindVisibleBlocks(scene.get(), &pose, &intrinsics, renderState.get());
  voxelVisualisationEngine->CreateExpectedDepths(scene.get(), &pose, &intrinsics, renderState.get());
  voxelVisualisationEngine->FindSurface(scene.get(), &pose, &intrinsics, renderState.get());

  const rigging::SimpleCamera camera = CameraPoseConverter::pose_to_camera(pose);
  depthVisualiser->render_depth(
    depthType, orx::GeometryUtil::to_itm(camera.p()), orx::GeometryUtil::to_itm(camera.n()),
    renderState.get(), settings->sceneParams.voxelSize, -1.0f, output
  );

  if(settings->deviceType == DEVICE_CUDA) output->UpdateHostFromDevice();
}

}
