/**
 * spaint: DepthVisualiser_CUDA.h
 */

#ifndef H_SPAINT_DEPTHVISUALISER_CUDA
#define H_SPAINT_DEPTHVISUALISER_CUDA

#include "../interface/DepthVisualiser.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to render a depth visualisation of an InfiniTAM scene using CUDA.
 */
class DepthVisualiser_CUDA : public DepthVisualiser
{
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void render_depth(const ITMLib::Objects::ITMRenderState *renderState, Vector3f cameraPosition, Vector3f cameraLookVector, float voxelSize, DepthType depthType,
                            ITMFloatImage *outputImage) const;
};

}

#endif
