/**
 * spaint: DepthVisualiser_CUDA.cu
 */

#include "visualisers/cuda/DepthVisualiser_CUDA.h"

#include "visualisers/shared/DepthVisualiser_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_render_depth(float *outRendering, Vector3f cameraPosition, Vector3f cameraLookVector, const Vector4f *ptsRay, Vector2i imgSize, float voxelSize, DepthVisualiser::DepthType depthType)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if(x >= imgSize.x || y >= imgSize.y) return;

  int locId = y * imgSize.x + x;
  Vector4f ptRay = ptsRay[locId];
  shade_pixel_depth(outRendering[locId], cameraPosition, cameraLookVector, ptRay.toVector3() * voxelSize, voxelSize, ptRay.w > 0, depthType);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void DepthVisualiser_CUDA::render_depth(ITMFloatImage *outputImage, const ITMLib::Objects::ITMRenderState *renderState, Vector3f cameraPosition, Vector3f cameraLookVector, float voxelSize, DepthType depthType) const
{
  Vector2i imgSize = outputImage->noDims;

  // Shade all the pixels in the image.
  dim3 cudaBlockSize(8, 8);
  dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
  ck_render_depth<<<gridSize,cudaBlockSize>>>(
    outputImage->GetData(MEMORYDEVICE_CUDA),
    cameraPosition,
    cameraLookVector,
    renderState->raycastResult->GetData(MEMORYDEVICE_CUDA),
    imgSize,
    voxelSize,
    depthType
  );
}

}

