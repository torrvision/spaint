/**
 * spaint: VOPFeatureCalculator_CUDA.cpp
 */

#include "features/cuda/VOPFeatureCalculator_CUDA.h"

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

#include "features/shared/VOPFeatureCalculator_Shared.h"

#define DEBUGGING 1

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_calculate_surface_normals(const Vector3s *voxelLocations, const int voxelLocationCount,
                                             const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                             Vector3f *surfaceNormals)
{
  int voxelLocationIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelLocationIndex < voxelLocationCount)
  {
    write_surface_normal(voxelLocationIndex, voxelLocations, voxelData, indexData, surfaceNormals);
  }
}

__global__ void ck_generate_coordinate_systems(const Vector3f *surfaceNormals, const int voxelLocationCount, Vector3f *xAxes, Vector3f *yAxes)
{
  int voxelLocationIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelLocationIndex < voxelLocationCount)
  {
    generate_coordinate_system(voxelLocationIndex, surfaceNormals, xAxes, yAxes);
  }
}

__global__ void ck_generate_rgb_patches(const Vector3s *voxelLocations, const int voxelLocationCount,
                                        const Vector3f *xAxes, const Vector3f *yAxes,
                                        const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                        size_t patchSize, float patchSpacing, size_t featureCount,
                                        float *features)
{
  int voxelLocationIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelLocationIndex < voxelLocationCount)
  {
    generate_rgb_patch(voxelLocationIndex, voxelLocations, xAxes, yAxes, voxelData, indexData, patchSize, patchSpacing, featureCount, features);
  }
}

//#################### CONSTRUCTORS ####################

VOPFeatureCalculator_CUDA::VOPFeatureCalculator_CUDA(size_t maxVoxelLocationCount, size_t patchSize, float patchSpacing)
: VOPFeatureCalculator(maxVoxelLocationCount, patchSize, patchSpacing)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VOPFeatureCalculator_CUDA::calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                                          const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const
{
  const int voxelLocationCount = static_cast<int>(voxelLocationsMB.dataSize);

  int threadsPerBlock = 256;
  int numBlocks = (voxelLocationCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_calculate_surface_normals<<<numBlocks,threadsPerBlock>>>(
    voxelLocationsMB.GetData(MEMORYDEVICE_CUDA),
    voxelLocationCount,
    voxelData,
    indexData,
    m_surfaceNormalsMB.GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  m_surfaceNormalsMB.UpdateHostFromDevice();
#endif
}

void VOPFeatureCalculator_CUDA::convert_patches_to_lab(ORUtils::MemoryBlock<float>& featuresMB) const
{
  // TODO
}

void VOPFeatureCalculator_CUDA::generate_coordinate_systems(int voxelLocationCount) const
{
  int threadsPerBlock = 256;
  int numBlocks = (voxelLocationCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_generate_coordinate_systems<<<numBlocks,threadsPerBlock>>>(
    m_surfaceNormalsMB.GetData(MEMORYDEVICE_CUDA),
    voxelLocationCount,
    m_xAxesMB.GetData(MEMORYDEVICE_CUDA),
    m_yAxesMB.GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  m_xAxesMB.UpdateHostFromDevice();
  m_yAxesMB.UpdateHostFromDevice();
#endif
}

void VOPFeatureCalculator_CUDA::generate_rgb_patches(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                                     const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                                     ORUtils::MemoryBlock<float>& featuresMB) const
{
  const int voxelLocationCount = static_cast<int>(voxelLocationsMB.dataSize);

  int threadsPerBlock = 256;
  int numBlocks = (voxelLocationCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_generate_rgb_patches<<<numBlocks,threadsPerBlock>>>(
    voxelLocationsMB.GetData(MEMORYDEVICE_CUDA),
    voxelLocationCount,
    m_xAxesMB.GetData(MEMORYDEVICE_CUDA),
    m_yAxesMB.GetData(MEMORYDEVICE_CUDA),
    voxelData,
    indexData,
    m_patchSize,
    m_patchSpacing,
    get_feature_count(),
    featuresMB.GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  featuresMB.UpdateHostFromDevice();
#endif
}

}
