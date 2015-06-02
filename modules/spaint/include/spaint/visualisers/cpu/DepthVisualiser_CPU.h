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
  virtual void render_depth(DepthType depthType, const Vector3f& cameraPosition, const Vector3f& cameraLookVector, const ITMLib::Objects::ITMRenderState *renderState,
                            float voxelSize, const ITMFloatImage_Ptr& outputImage) const;
};

}

#endif
