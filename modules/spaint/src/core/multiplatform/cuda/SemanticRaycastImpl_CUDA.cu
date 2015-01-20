/**
 * spaint: SemanticRaycastImpl_CUDA.cu
 */

#include "core/multiplatform/cuda/SemanticRaycastImpl_CUDA.h"

#include "core/multiplatform/shared/SemanticRaycastImpl_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

// IMPORTED FROM InfiniTAM - DO NOT CHANGE
template<class TVoxel, class TIndex>
__global__ void genericRaycast_device(Vector4f *out_ptsRay, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
  Vector2i imgSize, Matrix4f invM, Vector4f projParams, float oneOverVoxelSize, const Vector2f *minmaxdata, float mu)
{
  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x >= imgSize.x || y >= imgSize.y) return;

  int locId = x + y * imgSize.x;
  int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

  castRay<TVoxel, TIndex>(out_ptsRay[locId], x, y, voxelData, voxelIndex, invM, projParams, oneOverVoxelSize, mu, minmaxdata[locId2]);
}
// END

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
  Vector2i imgSize = outputImage->noDims;
  float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

  Matrix4f invM = pose->invM;
  Vector4f projParams = intrinsics->projectionParamsSimple.all;
  projParams.x = 1.0f / projParams.x;
  projParams.y = 1.0f / projParams.y;

  float mu = scene->sceneParams->mu;
  Vector3f lightSource = -Vector3f(invM.getColumn(2));

  Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CUDA);
  const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CUDA);
  Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CUDA);

  dim3 cudaBlockSize(8, 8);
  dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));

  genericRaycast_device<SpaintVoxel,ITMVoxelIndex> <<<gridSize, cudaBlockSize>>>(
    pointsRay,
    scene->localVBA.GetVoxelBlocks(),
    scene->index.getIndexData(),
    imgSize,
    invM,
    projParams,
    oneOverVoxelSize,
    minmaximg,
    mu
  );

  // Set up the label colours (quick hack).
  ORUtils::MemoryBlock<Vector3u> labelColours(4 * sizeof(Vector3u), true, true);
  Vector3u *labelColoursData = labelColours.GetData(MEMORYDEVICE_CPU);
  labelColoursData[0] = Vector3u(255, 255, 255);
  labelColoursData[1] = Vector3u(255, 0, 0);
  labelColoursData[2] = Vector3u(0, 255, 0);
  labelColoursData[3] = Vector3u(0, 0, 255);
  labelColours.UpdateDeviceFromHost();

  renderSemantic_device<<<gridSize, cudaBlockSize>>>(
    outRendering,
    pointsRay,
    scene->localVBA.GetVoxelBlocks(),
    scene->index.getIndexData(),
    imgSize,
    lightSource,
    labelColours.GetData(MEMORYDEVICE_CUDA)
  );
}

}
