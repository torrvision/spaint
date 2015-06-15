/**
 * spaint: SemanticVisualiser_CUDA.cu
 */

#include "visualisers/cuda/SemanticVisualiser_CUDA.h"

#include "visualisers/shared/SemanticVisualiser_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_render_semantic(Vector4u *outRendering, const Vector4f *ptsRay, const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex,
                                   Vector2i imgSize, Vector3u *labelColours, Vector3f viewerPos, Vector3f lightPos, bool usePhong, float labelAlpha)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= imgSize.x || y >= imgSize.y) return;

  int locId = y * imgSize.x + x;
  Vector4f ptRay = ptsRay[locId];
  shade_pixel_semantic(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, labelColours, viewerPos, lightPos, usePhong, labelAlpha);
}

//#################### CONSTRUCTORS ####################

SemanticVisualiser_CUDA::SemanticVisualiser_CUDA(const std::vector<Vector3u>& labelColours)
: SemanticVisualiser(labelColours)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SemanticVisualiser_CUDA::render(const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, const ITMLib::Objects::ITMPose *pose,
                                     const ITMLib::Objects::ITMIntrinsics *intrinsics, const ITMLib::Objects::ITMRenderState *renderState,
                                     bool usePhong, float labelAlpha, ITMUChar4Image *outputImage) const
{
  // Calculate the light and viewer positions in voxel coordinates (the same coordinate space as the raycast results).
  const float voxelSize = scene->sceneParams->voxelSize;
  Vector3f lightPos = Vector3f(0.0f, -10.0f, -10.0f) / voxelSize;
  Vector3f viewerPos = Vector3f(pose->GetInvM().getColumn(3)) / voxelSize;

  // Shade all of the pixels in the image.
  Vector2i imgSize = outputImage->noDims;

  dim3 cudaBlockSize(8, 8);
  dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));

  ck_render_semantic<<<gridSize,cudaBlockSize>>>(
    outputImage->GetData(MEMORYDEVICE_CUDA),
    renderState->raycastResult->GetData(MEMORYDEVICE_CUDA),
    scene->localVBA.GetVoxelBlocks(),
    scene->index.getIndexData(),
    imgSize,
    m_labelColoursMB.GetData(MEMORYDEVICE_CUDA),
    viewerPos,
    lightPos,
    usePhong,
    labelAlpha
  );
}

}
