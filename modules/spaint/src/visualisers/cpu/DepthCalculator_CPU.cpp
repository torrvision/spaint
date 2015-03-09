/**
 * spaint: DepthCalculator_CPU.cpp
 */

#include "visualisers/cpu/DepthCalculator_CPU.h"

#include "visualisers/shared/DepthCalculator_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS #################### 

void DepthCalculator_CPU::render_depth(ITMFloatImage *outputImage, const ITMLib::Objects::ITMRenderState *renderState, Vector3f cameraPosition, Vector3f cameraLookVector, float voxelSize, DepthType depthType) const
{
  int imgSize = outputImage->noDims.x * outputImage->noDims.y;

  // Get relevant image pointers.
  float *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
  const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < imgSize; ++locId)
  {
    Vector4f ptRay = pointsRay[locId];
    shade_pixel_depth(outRendering[locId], cameraPosition, cameraLookVector, ptRay.toVector3(), voxelSize, ptRay.w > 0, depthType);
  }
}

}
