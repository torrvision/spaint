/**
 * spaint: VOPFeatureCalculator_CPU.cpp
 */

#include "features/cpu/VOPFeatureCalculator_CPU.h"

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

namespace spaint {

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VOPFeatureCalculator_CPU::calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB,
                                                         const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const
{
  const int labelCount = voxelCountsForLabelsMB.dataSize;
  Vector3f *surfaceNormals = m_surfaceNormalsMB.GetData(MEMORYDEVICE_CPU);
  const unsigned int *voxelCountsForLabels = voxelCountsForLabelsMB.GetData(MEMORYDEVICE_CPU);
  const Vector3s *voxelLocations = voxelLocationsMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < m_maxVoxelsPerLabel; ++i)
  {
    for(int k = 0; k < labelCount; ++k)
    {
      if(i < voxelCountsForLabels[k])
      {
        surfaceNormals[k * m_maxVoxelsPerLabel + i] = computeSingleNormalFromSDF(voxelData, indexData, voxelLocations[k * m_maxVoxelsPerLabel + i].toFloat());
      }
    }
  }
}

}
