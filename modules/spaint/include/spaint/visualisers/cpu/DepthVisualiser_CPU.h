/**
 * spaint: DepthVisualiser_CPU.h
 */

#ifndef H_SPAINT_DEPTHVISUALISER_CPU
#define H_SPAINT_DEPTHVISUALISER_CPU

#include "../interface/DepthVisualiser.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to render a depth visualisation of an InfiniTAM scene using the CPU.
 */
class DepthVisualiser_CPU : public DepthVisualiser
{
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void render_depth(const ITMLib::Objects::ITMRenderState *renderState, Vector3f cameraPosition, Vector3f cameraLookVector, float voxelSize, DepthType depthType,
                            ITMFloatImage *outputImage) const;
};

}

#endif
