/**
 * itmx: DepthVisualisationUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_DEPTHVISUALISATIONUTIL
#define H_ITMX_DEPTHVISUALISATIONUTIL

#include "../base/ITMImagePtrTypes.h"
#include "../base/ITMObjectPtrTypes.h"
#include "interface/DepthVisualiser.h"

namespace itmx {

/**
 * \brief This struct provides a utility function that can render a synthetic depth image of a voxel scene.
 */
template <typename VoxelType, typename IndexType>
struct DepthVisualisationUtil
{
  //#################### TYPEDEFS ####################

  typedef ITMLib::ITMScene<VoxelType,IndexType> Scene;
  typedef boost::shared_ptr<const Scene> Scene_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMVisualisationEngine<VoxelType,IndexType> > VoxelVisualisationEngine_CPtr;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Generates a synthetic depth image of a voxel scene from the specified pose.
   *
   * \note  This produces a floating-point depth image whose format matches that used by InfiniTAM,
   *        as opposed to a colourised depth image that is suitable for showing to the user.
   *
   * \param output                    The location into which to put the output image.
   * \param scene                     The scene to visualise.
   * \param pose                      The pose from which to visualise the scene.
   * \param intrinsics                The camera intrinsics to use when visualising the scene.
   * \param renderState               The render state to use for intermediate storage (can be null, in which case a new one will be created).
   * \param depthType                 The type of depth calculation to use.
   * \param voxelVisualisationEngine  The InfiniTAM engine to use for rendering a voxel scene.
   * \param depthVisualiser           The depth visualiser.
   * \param settings                  The settings to use for InfiniTAM.
   */
  static void generate_depth_from_voxels(const ITMFloatImage_Ptr& output, const Scene_CPtr& scene, const ORUtils::SE3Pose& pose,
                                         const ITMLib::ITMIntrinsics& intrinsics, VoxelRenderState_Ptr& renderState, DepthVisualiser::DepthType depthType,
                                         const VoxelVisualisationEngine_CPtr& voxelVisualisationEngine, const itmx::DepthVisualiser_CPtr& depthVisualiser,
                                         const Settings_CPtr& settings);
};

}

#endif
