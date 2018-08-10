/**
 * itmx: DepthVisualiser_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "visualisation/cuda/DepthVisualiser_CUDA.h"

#include "visualisation/shared/DepthVisualiser_Shared.h"

namespace itmx {

//#################### CUDA KERNELS ####################

__global__ void ck_render_depth(float *outRendering, const Vector4f *ptsRay, Vector3f cameraPosition, Vector3f cameraLookVector,
                                Vector2i imgSize, float voxelSize, float invalidDepthValue, DepthVisualiser::DepthType depthType)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if(x >= imgSize.x || y >= imgSize.y) return;

  int locId = y * imgSize.x + x;
  Vector4f ptRay = ptsRay[locId];
  shade_pixel_depth(outRendering[locId], ptRay.toVector3() * voxelSize, ptRay.w > 0, cameraPosition, cameraLookVector, voxelSize, invalidDepthValue, depthType);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void DepthVisualiser_CUDA::render_depth(DepthType depthType, const Vector3f& cameraPosition, const Vector3f& cameraLookVector, const ITMLib::ITMRenderState *renderState,
                                        float voxelSize, float invalidDepthValue, const ORFloatImage_Ptr& outputImage) const
{
  if(!renderState) return;

  Vector2i imgSize = outputImage->noDims;

  // Shade all of the pixels in the image.
  dim3 cudaBlockSize(8, 8);
  dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));

  ck_render_depth<<<gridSize,cudaBlockSize>>>(
    outputImage->GetData(MEMORYDEVICE_CUDA),
    renderState->raycastResult->GetData(MEMORYDEVICE_CUDA),
    cameraPosition,
    cameraLookVector,
    imgSize,
    voxelSize,
    invalidDepthValue,
    depthType
  );
}

}
