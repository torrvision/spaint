/**
 * spaint: LabelPropagator_Shared.h
 */

#ifndef H_SPAINT_LABELPROPAGATOR_SHARED
#define H_SPAINT_LABELPROPAGATOR_SHARED

#include "../../markers/shared/VoxelMarker_Shared.h"

namespace spaint {

_CPU_AND_GPU_CODE_
bool should_propagate_from_neighbour(int neighbourX, int neighbourY, int x, int y)
{
  return true;
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void propagate_from_neighbours(int voxelIndex, int width, int height, SpaintVoxel::PackedLabel label,
                                      const Vector4f *raycastResult, const Vector3f *surfaceNormals,
                                      SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData)
{
  int x = voxelIndex % width;
  int y = voxelIndex / width;

  Vector3s loc = raycastResult[voxelIndex].toVector3().toShortRound();

  if(should_propagate_from_neighbour(x - 1, y, x, y) ||
     should_propagate_from_neighbour(x + 1, y, x, y) ||
     should_propagate_from_neighbour(x, y - 1, x, y) ||
     should_propagate_from_neighbour(x, y + 1, x, y))
  {
    mark_voxel(loc, label, NULL, voxelData, indexData);
  }
}

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
