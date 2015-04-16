/**
 * spaint: VOPFeatureCalculator_CUDA.cpp
 */

#include "features/cuda/VOPFeatureCalculator_CUDA.h"

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

#include "features/shared/VOPFeatureCalculator_Shared.h"

#define DEBUGGING 1

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_calculate_surface_normals(const Vector3s *voxelLocations, const unsigned int *voxelCountsForLabels,
                                             const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                             const int voxelLocationCount, const int maxVoxelsPerLabel,
                                             Vector3f *surfaceNormals)
{
  int voxelLocationIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelLocationIndex < voxelLocationCount)
  {
    write_surface_normal(voxelLocationIndex, voxelLocations, voxelCountsForLabels, voxelData, indexData, maxVoxelsPerLabel, surfaceNormals);
  }
}

//#################### CONSTRUCTORS ####################

VOPFeatureCalculator_CUDA::VOPFeatureCalculator_CUDA(int maxLabelCount, int maxVoxelsPerLabel)
: VOPFeatureCalculator(maxLabelCount, maxVoxelsPerLabel)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VOPFeatureCalculator_CUDA::calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB,
                                                          const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const
{
  const int voxelLocationCount = voxelLocationsMB.dataSize;

  int threadsPerBlock = 256;
  int numBlocks = (voxelLocationCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_calculate_surface_normals<<<numBlocks,threadsPerBlock>>>(
    voxelLocationsMB.GetData(MEMORYDEVICE_CUDA),
    voxelCountsForLabelsMB.GetData(MEMORYDEVICE_CUDA),
    voxelData,
    indexData,
    voxelLocationCount,
    m_maxVoxelsPerLabel,
    m_surfaceNormalsMB.GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  m_surfaceNormalsMB.UpdateHostFromDevice();
#endif
}

}
