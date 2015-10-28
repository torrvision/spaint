/**
 * spaint: DepthVisualiser_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "visualisers/cpu/DepthVisualiser_CPU.h"

#include "visualisers/shared/DepthVisualiser_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void DepthVisualiser_CPU::render_depth(DepthType depthType, const Vector3f& cameraPosition, const Vector3f& cameraLookVector, const ITMLib::ITMRenderState *renderState,
                                       float voxelSize, float invalidDepthValue, const ITMFloatImage_Ptr& outputImage) const
{
  int imgSize = outputImage->noDims.x * outputImage->noDims.y;
  float *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
  const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < imgSize; ++locId)
  {
    Vector4f ptRay = pointsRay[locId];
    shade_pixel_depth(outRendering[locId], ptRay.toVector3() * voxelSize, ptRay.w > 0, cameraPosition, cameraLookVector, voxelSize, invalidDepthValue, depthType);
  }
}

}
