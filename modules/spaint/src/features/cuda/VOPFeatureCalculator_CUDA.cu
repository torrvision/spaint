/**
 * spaint: VOPFeatureCalculator_CUDA.cpp
 */

#include "features/cuda/VOPFeatureCalculator_CUDA.h"

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

#include "features/shared/VOPFeatureCalculator_Shared.h"

#define DEBUGGING 0

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_calculate_surface_normals(const Vector3s *voxelLocations, const int voxelLocationCount,
                                             const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                             Vector3f *surfaceNormals, const size_t featureCount, float *features)
{
  int voxelLocationIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelLocationIndex < voxelLocationCount)
  {
    write_surface_normal(voxelLocationIndex, voxelLocations, voxelData, indexData, surfaceNormals, featureCount, features);
  }
}

__global__ void ck_convert_patches_to_lab(const int voxelLocationCount, const size_t featureCount, float *features)
{
  int voxelLocationIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelLocationIndex < voxelLocationCount)
  {
    convert_patch_to_lab(voxelLocationIndex, featureCount, features);
  }
}

__global__ void ck_fill_in_heights(const Vector3s *voxelLocations, const int voxelLocationCount, const size_t featureCount, float *features)
{
  int voxelLocationIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelLocationIndex < voxelLocationCount)
  {
    fill_in_height(voxelLocationIndex, voxelLocations, featureCount, features);
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
                                        const size_t patchSize, const float patchSpacing, const size_t featureCount,
                                        float *features)
{
  int voxelLocationIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelLocationIndex < voxelLocationCount)
  {
    generate_rgb_patch(voxelLocationIndex, voxelLocations, xAxes, yAxes, voxelData, indexData, patchSize, patchSpacing, featureCount, features);
  }
}

__global__ void ck_update_coordinate_systems(const int voxelLocationCount, const float *features, const size_t featureCount,
                                             const size_t patchSize, const size_t binCount, Vector3f *xAxes, Vector3f *yAxes)
{
  // Note: We declare these shared arrays with fixed sizes here for simplicity. The sizes will need to be changed
  //       if we ever want to use a histogram with more than 64 bins or VOP patches that are bigger than 16 x 16.
  __shared__ float histogram[64];
  __shared__ float intensities[256];

  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  int voxelLocationIndex = tid / (patchSize * patchSize);

  // Convert the voxel's RGB patch to an intensity patch.
  compute_intensities_for_patch(tid, features, featureCount, patchSize, intensities);
  __syncthreads();

  // Compute a histogram of oriented gradients from the intensity patch for the voxel.
  compute_histogram_for_patch(tid, patchSize, intensities, binCount, histogram);
  __syncthreads();

  // Calculate the dominant orientation for the voxel and rotate its coordinate system to align with that as necessary.
  update_coordinate_system(tid, patchSize * patchSize, histogram, binCount, &xAxes[voxelLocationIndex], &yAxes[voxelLocationIndex]);
}

//#################### CONSTRUCTORS ####################

VOPFeatureCalculator_CUDA::VOPFeatureCalculator_CUDA(size_t maxVoxelLocationCount, size_t patchSize, float patchSpacing, size_t binCount)
: VOPFeatureCalculator(maxVoxelLocationCount, patchSize, patchSpacing, binCount)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VOPFeatureCalculator_CUDA::calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                                          const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                                          ORUtils::MemoryBlock<float>& featuresMB) const
{
  const int voxelLocationCount = static_cast<int>(voxelLocationsMB.dataSize);

  int threadsPerBlock = 256;
  int numBlocks = (voxelLocationCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_calculate_surface_normals<<<numBlocks,threadsPerBlock>>>(
    voxelLocationsMB.GetData(MEMORYDEVICE_CUDA),
    voxelLocationCount,
    voxelData,
    indexData,
    m_surfaceNormalsMB->GetData(MEMORYDEVICE_CUDA),
    get_feature_count(),
    featuresMB.GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  m_surfaceNormalsMB->UpdateHostFromDevice();
  featuresMB.UpdateHostFromDevice();
#endif
}

void VOPFeatureCalculator_CUDA::convert_patches_to_lab(int voxelLocationCount, ORUtils::MemoryBlock<float>& featuresMB) const
{
  int threadsPerBlock = 256;
  int numBlocks = (voxelLocationCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_convert_patches_to_lab<<<numBlocks,threadsPerBlock>>>(
    voxelLocationCount,
    get_feature_count(),
    featuresMB.GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  featuresMB.UpdateHostFromDevice();
#endif
}

void VOPFeatureCalculator_CUDA::fill_in_heights(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, ORUtils::MemoryBlock<float>& featuresMB) const
{
  const int voxelLocationCount = static_cast<int>(voxelLocationsMB.dataSize);

  int threadsPerBlock = 256;
  int numBlocks = (voxelLocationCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_fill_in_heights<<<numBlocks,threadsPerBlock>>>(
    voxelLocationsMB.GetData(MEMORYDEVICE_CUDA),
    voxelLocationCount,
    get_feature_count(),
    featuresMB.GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  featuresMB.UpdateHostFromDevice();
#endif
}

void VOPFeatureCalculator_CUDA::generate_coordinate_systems(int voxelLocationCount) const
{
  int threadsPerBlock = 256;
  int numBlocks = (voxelLocationCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_generate_coordinate_systems<<<numBlocks,threadsPerBlock>>>(
    m_surfaceNormalsMB->GetData(MEMORYDEVICE_CUDA),
    voxelLocationCount,
    m_xAxesMB->GetData(MEMORYDEVICE_CUDA),
    m_yAxesMB->GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  m_xAxesMB->UpdateHostFromDevice();
  m_yAxesMB->UpdateHostFromDevice();
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
    m_xAxesMB->GetData(MEMORYDEVICE_CUDA),
    m_yAxesMB->GetData(MEMORYDEVICE_CUDA),
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

void VOPFeatureCalculator_CUDA::update_coordinate_systems(int voxelLocationCount, const ORUtils::MemoryBlock<float>& featuresMB) const
{
  int threadsPerBlock = static_cast<int>(m_patchSize * m_patchSize);
  int numBlocks = voxelLocationCount;

  ck_update_coordinate_systems<<<numBlocks,threadsPerBlock>>>(
    voxelLocationCount,
    featuresMB.GetData(MEMORYDEVICE_CUDA),
    get_feature_count(),
    m_patchSize,
    m_binCount,
    m_xAxesMB->GetData(MEMORYDEVICE_CUDA),
    m_yAxesMB->GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  m_xAxesMB->UpdateHostFromDevice();
  m_yAxesMB->UpdateHostFromDevice();
#endif
}

}
