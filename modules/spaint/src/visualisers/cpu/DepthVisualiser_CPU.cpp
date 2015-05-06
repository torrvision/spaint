/**
 * spaint: DepthVisualiser_CPU.cpp
 */

#include "visualisers/cpu/DepthVisualiser_CPU.h"

#include "visualisers/shared/DepthVisualiser_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void DepthVisualiser_CPU::render_depth(const ITMLib::Objects::ITMRenderState *renderState, Vector3f cameraPosition, Vector3f cameraLookVector, float voxelSize, DepthType depthType,
                                       ITMFloatImage *outputImage) const
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
    shade_pixel_depth(outRendering[locId], cameraPosition, cameraLookVector, ptRay.toVector3() * voxelSize, voxelSize, ptRay.w > 0, depthType);
  }
}

}
