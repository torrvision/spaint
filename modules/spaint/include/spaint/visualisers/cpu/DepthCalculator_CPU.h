/**
 * spaint: DepthCalculator_CPU.h
 */

#ifndef H_SPAINT_DEPTHVISUALISER_CPU
#define H_SPAINT_DEPTHVISUALISER_CPU

#include "../interface/DepthCalculator.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to render a depth visualisation of an InfiniTAM scene using the CPU.
 */
class DepthCalculator_CPU : public DepthCalculator
{
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void render_depth(ITMFloatImage *outputImage, const ITMLib::Objects::ITMRenderState *renderState, const rigging::SimpleCamera *camera, float voxelSize, DepthType depthType) const;
};

}

#endif
