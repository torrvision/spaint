/**
 * spaint: VOPFeatureCalculator_Shared.h
 */

#ifndef H_SPAINT_VOPFEATURECALCULATOR_SHARED
#define H_SPAINT_VOPFEATURECALCULATOR_SHARED

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

#include "../../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void write_surface_normal(int voxelLocationIndex, const Vector3s *voxelLocations, const unsigned int *voxelCountsForLabels,
                                 const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData, const int maxVoxelsPerLabel,
                                 Vector3f *surfaceNormals)
{
  unsigned int label = voxelLocationIndex / maxVoxelsPerLabel;
  unsigned int offset = voxelLocationIndex % maxVoxelsPerLabel;
  if(offset < voxelCountsForLabels[label])
  {
    surfaceNormals[voxelLocationIndex] = computeSingleNormalFromSDF(voxelData, indexData, voxelLocations[voxelLocationIndex].toFloat());
  }
}

}

#endif
