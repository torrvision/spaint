/**
 * spaint: VOPFeatureCalculator_CPU.cpp
 */

#include "features/cpu/VOPFeatureCalculator_CPU.h"

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

#include "features/shared/VOPFeatureCalculator_Shared.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

VOPFeatureCalculator_CPU::VOPFeatureCalculator_CPU(size_t maxVoxelLocationCount, size_t patchSize, float patchSpacing)
: VOPFeatureCalculator(maxVoxelLocationCount, patchSize, patchSpacing)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VOPFeatureCalculator_CPU::calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                                         const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const
{
  Vector3f *surfaceNormals = m_surfaceNormalsMB.GetData(MEMORYDEVICE_CPU);
  const Vector3s *voxelLocations = voxelLocationsMB.GetData(MEMORYDEVICE_CPU);
  const int voxelLocationCount = static_cast<int>(voxelLocationsMB.dataSize);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelLocationIndex = 0; voxelLocationIndex < voxelLocationCount; ++voxelLocationIndex)
  {
    write_surface_normal(voxelLocationIndex, voxelLocations, voxelData, indexData, surfaceNormals);
  }
}

void VOPFeatureCalculator_CPU::convert_patches_to_lab(int voxelLocationCount, ORUtils::MemoryBlock<float>& featuresMB) const
{
  const size_t featureCount = get_feature_count();
  float *features = featuresMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelLocationIndex = 0; voxelLocationIndex < voxelLocationCount; ++voxelLocationIndex)
  {
    convert_patch_to_lab(voxelLocationIndex, featureCount, features);
  }
}

void VOPFeatureCalculator_CPU::generate_coordinate_systems(int voxelLocationCount) const
{
  const Vector3f *surfaceNormals = m_surfaceNormalsMB.GetData(MEMORYDEVICE_CPU);
  Vector3f *xAxes = m_xAxesMB.GetData(MEMORYDEVICE_CPU);
  Vector3f *yAxes = m_yAxesMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelLocationIndex = 0; voxelLocationIndex < voxelLocationCount; ++voxelLocationIndex)
  {
    generate_coordinate_system(voxelLocationIndex, surfaceNormals, xAxes, yAxes);
  }
}

void VOPFeatureCalculator_CPU::generate_rgb_patches(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                                    const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                                    ORUtils::MemoryBlock<float>& featuresMB) const
{
  const size_t featureCount = get_feature_count();
  float *features = featuresMB.GetData(MEMORYDEVICE_CPU);
  const Vector3f *xAxes = m_xAxesMB.GetData(MEMORYDEVICE_CPU);
  const Vector3f *yAxes = m_yAxesMB.GetData(MEMORYDEVICE_CPU);
  const Vector3s *voxelLocations = voxelLocationsMB.GetData(MEMORYDEVICE_CPU);
  const int voxelLocationCount = static_cast<int>(voxelLocationsMB.dataSize);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelLocationIndex = 0; voxelLocationIndex < voxelLocationCount; ++voxelLocationIndex)
  {
    generate_rgb_patch(voxelLocationIndex, voxelLocations, xAxes, yAxes, voxelData, indexData, m_patchSize, m_patchSpacing, featureCount, features);
  }
}

}
