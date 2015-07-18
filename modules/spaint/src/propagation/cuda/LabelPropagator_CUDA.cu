/**
 * spaint: LabelPropagator_CUDA.cu
 */

#include "propagation/cuda/LabelPropagator_CUDA.h"

#include "propagation/shared/LabelPropagator_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_calculate_normals(const Vector4f *raycastResultData, int raycastResultSize,
                                     const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                     Vector3f *surfaceNormals)
{
  int voxelIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelIndex < raycastResultSize)
  {
    write_surface_normal(voxelIndex, raycastResultData, voxelData, indexData, surfaceNormals);
  }
}

__global__ void ck_perform_propagation()
{
  // TODO
}

//#################### CONSTRUCTORS ####################

LabelPropagator_CUDA::LabelPropagator_CUDA(size_t raycastResultSize)
: LabelPropagator(raycastResultSize)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void LabelPropagator_CUDA::calculate_normals(const ITMFloat4Image *raycastResult, const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const
{
  const int raycastResultSize = static_cast<int>(raycastResult->dataSize);

  int threadsPerBlock = 256;
  int numBlocks = (raycastResultSize + threadsPerBlock - 1) / threadsPerBlock;

  ck_calculate_normals<<<numBlocks,threadsPerBlock>>>(
    raycastResult->GetData(MEMORYDEVICE_CUDA),
    raycastResultSize,
    scene->localVBA.GetVoxelBlocks(),
    scene->index.getIndexData(),
    m_surfaceNormalsMB->GetData(MEMORYDEVICE_CUDA)
  );
}

void LabelPropagator_CUDA::perform_propagation(SpaintVoxel::Label label, const ITMFloat4Image *raycastResult,
                                               ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const
{
  // TODO
}

}
