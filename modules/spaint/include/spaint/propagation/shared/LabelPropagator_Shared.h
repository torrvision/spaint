/**
 * spaint: LabelPropagator_Shared.h
 */

#ifndef H_SPAINT_LABELPROPAGATOR_SHARED
#define H_SPAINT_LABELPROPAGATOR_SHARED

#include "../../markers/shared/VoxelMarker_Shared.h"

namespace spaint {

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline bool should_propagate_from_neighbour(int neighbourX, int neighbourY, int width, int height, SpaintVoxel::PackedLabel label,
                                            const Vector3f& loc, const Vector4f *raycastResult, const Vector3f *surfaceNormals,
                                            const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData)
{
  if(neighbourX < 0 || neighbourX >= width || neighbourY < 0 || neighbourY >= height) return false;

  Vector3f neighbourLoc = raycastResult[neighbourY * width + neighbourX].toVector3();

  bool foundPoint;
  const SpaintVoxel neighbourVoxel = readVoxel(voxelData, indexData, neighbourLoc.toIntRound(), foundPoint);
  if(!foundPoint) return false;

  Vector3f posOffset = neighbourLoc - loc;
  float distanceSquared = dot(posOffset, posOffset);

  //return distanceSquared < 10.0f;
  return neighbourVoxel.packedLabel == label;
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

  Vector3f loc = raycastResult[voxelIndex].toVector3();

  if(should_propagate_from_neighbour(x - 1, y, width, height, label, loc, raycastResult, surfaceNormals, voxelData, indexData) ||
     should_propagate_from_neighbour(x + 1, y, width, height, label, loc, raycastResult, surfaceNormals, voxelData, indexData) ||
     should_propagate_from_neighbour(x, y - 1, width, height, label, loc, raycastResult, surfaceNormals, voxelData, indexData) ||
     should_propagate_from_neighbour(x, y + 1, width, height, label, loc, raycastResult, surfaceNormals, voxelData, indexData))
  {
    mark_voxel(loc.toShortRound(), label, NULL, voxelData, indexData);
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
