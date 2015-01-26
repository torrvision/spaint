/**
 * spaint: SemanticRaycastImpl_CUDA.cu
 */

#include "core/multiplatform/cuda/SemanticRaycastImpl_CUDA.h"

#include "core/multiplatform/shared/SemanticRaycastImpl_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void renderSemantic_device(Vector4u *outRendering, const Vector4f *ptsRay, const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex,
                                      Vector2i imgSize, Vector3f lightSource, Vector3u *labelColours)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= imgSize.x || y >= imgSize.y) return;

  int locId = y * imgSize.x + x;
  Vector4f ptRay = ptsRay[locId];
  processPixelSemantic(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource, labelColours);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SemanticRaycastImpl_CUDA::render(const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, const ITMLib::Objects::ITMPose *pose,
                                      const ITMLib::Objects::ITMIntrinsics *intrinsics, const ITMLib::Objects::ITMRenderState *renderState,
                                      ITMUChar4Image *outputImage) const
{
  // Set up the label colours (quick hack).
  ORUtils::MemoryBlock<Vector3u> labelColours(4 * sizeof(Vector3u), true, true);
  Vector3u *labelColoursData = labelColours.GetData(MEMORYDEVICE_CPU);
  labelColoursData[0] = Vector3u(255, 255, 255);
  labelColoursData[1] = Vector3u(255, 0, 0);
  labelColoursData[2] = Vector3u(0, 255, 0);
  labelColoursData[3] = Vector3u(0, 0, 255);
  labelColours.UpdateDeviceFromHost();

  Vector2i imgSize = outputImage->noDims;
  dim3 cudaBlockSize(8, 8);
  dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
  renderSemantic_device<<<gridSize, cudaBlockSize>>>(
    outputImage->GetData(MEMORYDEVICE_CUDA),
    renderState->raycastResult->GetData(MEMORYDEVICE_CUDA),
    scene->localVBA.GetVoxelBlocks(),
    scene->index.getIndexData(),
    imgSize,
    -Vector3f(pose->invM.getColumn(2)),
    labelColours.GetData(MEMORYDEVICE_CUDA)
  );
}

}
