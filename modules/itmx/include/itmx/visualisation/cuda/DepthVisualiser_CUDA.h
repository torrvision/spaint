/**
 * itmx: DepthVisualiser_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_ITMX_DEPTHVISUALISER_CUDA
#define H_ITMX_DEPTHVISUALISER_CUDA

#include "../interface/DepthVisualiser.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to render a depth visualisation of an InfiniTAM scene using CUDA.
 */
class DepthVisualiser_CUDA : public DepthVisualiser
{
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void render_depth(DepthType depthType, const Vector3f& cameraPosition, const Vector3f& cameraLookVector, const ITMLib::ITMRenderState *renderState,
                            float voxelSize, float invalidDepthValue, const ITMFloatImage_Ptr& outputImage) const;
};

}

#endif
