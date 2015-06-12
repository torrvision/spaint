/**
 * spaint: UniformVoxelSampler_CPU.cpp
 */

#include "sampling/cpu/UniformVoxelSampler_CPU.h"

#include "sampling/shared/UniformVoxelSampler_Shared.h"

namespace spaint {

//#################### PRIVATE MEMBER FUNCTIONS ####################

void UniformVoxelSampler_CPU::write_sampled_voxel_locations(const ITMFloat4Image *raycastResult, size_t voxelsToSample,
                                                            ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const
{
  const Vector4f *raycastResultData = raycastResult->GetData(MEMORYDEVICE_CPU);
  const int *sampledVoxelIndices = m_sampledVoxelIndicesMB.GetData(MEMORYDEVICE_CPU);
  Vector3s *sampledVoxelLocations = sampledVoxelLocationsMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int tid = 0; tid < static_cast<int>(voxelsToSample); ++tid)
  {
    write_sampled_voxel_location(tid, raycastResultData, sampledVoxelIndices, sampledVoxelLocations);
  }
}

}
