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
                                             const int voxelLocationCount, const size_t maxVoxelsPerLabel,
                                             Vector3f *surfaceNormals)
{
  int voxelLocationIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelLocationIndex < voxelLocationCount)
  {
    write_surface_normal(voxelLocationIndex, voxelLocations, voxelCountsForLabels, voxelData, indexData, maxVoxelsPerLabel, surfaceNormals);
  }
}

__global__ void ck_generate_coordinate_systems(const Vector3f *surfaceNormals, const unsigned int *voxelCountsForLabels,
                                               const int voxelLocationCount, const size_t maxVoxelsPerLabel,
                                               Vector3f *xAxes, Vector3f *yAxes)
{
  int voxelLocationIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelLocationIndex < voxelLocationCount)
  {
    generate_coordinate_system(voxelLocationIndex, surfaceNormals, voxelCountsForLabels, maxVoxelsPerLabel, xAxes, yAxes);
  }
}

//#################### CONSTRUCTORS ####################

VOPFeatureCalculator_CUDA::VOPFeatureCalculator_CUDA(size_t maxLabelCount, size_t maxVoxelsPerLabel, size_t patchSize, float patchSpacing)
: VOPFeatureCalculator(maxLabelCount, maxVoxelsPerLabel, patchSize, patchSpacing)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VOPFeatureCalculator_CUDA::calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                                          const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB,
                                                          const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const
{
  const int voxelLocationCount = static_cast<int>(voxelLocationsMB.dataSize);

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

void VOPFeatureCalculator_CUDA::generate_coordinate_systems(const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const
{
  const int voxelLocationCount = static_cast<int>(m_surfaceNormalsMB.dataSize);

  int threadsPerBlock = 256;
  int numBlocks = (voxelLocationCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_generate_coordinate_systems<<<numBlocks,threadsPerBlock>>>(
    m_surfaceNormalsMB.GetData(MEMORYDEVICE_CUDA),
    voxelCountsForLabelsMB.GetData(MEMORYDEVICE_CUDA),
    voxelLocationCount,
    m_maxVoxelsPerLabel,
    m_xAxesMB.GetData(MEMORYDEVICE_CUDA),
    m_yAxesMB.GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  m_xAxesMB.UpdateHostFromDevice();
  m_yAxesMB.UpdateHostFromDevice();
#endif
}

}
