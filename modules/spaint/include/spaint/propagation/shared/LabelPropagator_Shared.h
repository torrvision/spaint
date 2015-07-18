/**
 * spaint: LabelPropagator_Shared.h
 */

#ifndef H_SPAINT_LABELPROPAGATOR_SHARED
#define H_SPAINT_LABELPROPAGATOR_SHARED

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

namespace spaint {

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void write_surface_normal(int voxelIndex, const Vector4f *raycastResult,
                                 const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                 Vector3f *surfaceNormals)
{
  Vector4f loc = raycastResult[voxelIndex];

  // If the voxel is valid, compute its actual surface normal; otherwise, use a default.
  Vector3f n(0.0f, 0.0f, 0.0f);
  if(loc.w > 0)
  {
    n = computeSingleNormalFromSDF(voxelData, indexData, loc.toVector3());
  }

  // Write the normal into the surface normals array.
  surfaceNormals[voxelIndex] = n;
}

}

#endif
