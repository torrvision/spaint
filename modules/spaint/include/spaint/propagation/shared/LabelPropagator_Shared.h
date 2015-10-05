/**
 * spaint: LabelPropagator_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_LABELPROPAGATOR_SHARED
#define H_SPAINT_LABELPROPAGATOR_SHARED

#include "../../markers/shared/VoxelMarker_Shared.h"
#include "../../util/ColourConversion_Shared.h"

namespace spaint {

/**
 * \brief Determines whether or not the specified label should be extrapolated from a specified neighbouring voxel to the voxel of interest.
 *
 * \param neighbourX                        The x coordinate of the neighbour in the raycast result.
 * \param neighbourY                        The y coordinate of the neighbour in the raycast result.
 * \param width                             The width of the raycast result.
 * \param height                            The height of the raycast result.
 * \param label                             The label being propagated.
 * \param loc                               The position of the voxel of interest in the scene.
 * \param normal                            The surface normal of the voxel of interest.
 * \param colour                            The RGB colour of the voxel of interest.
 * \param raycastResult                     The raycast result.
 * \param surfaceNormals                    The surface normals for the voxels in the raycast result.
 * \param voxelData                         The scene's voxel data.
 * \param indexData                         The scene's index data.
 * \param maxAngleBetweenNormals            The largest angle allowed between the normals of the neighbour and the voxel of interest if extrapolation is to occur.
 * \param maxSquaredDistanceBetweenColours  The maximum squared distance allowed between the colours of the neighbour and the voxel of interest if extrapolation is to occur.
 * \param maxSquaredDistanceBetweenVoxels   The maximum squared distance allowed between the positions of the neighbour and the voxel of interest if extrapolation is to occur.
 * \return                                  true, if the label should be propagated from the neighbour, or false otherwise.
 */
_CPU_AND_GPU_CODE_
inline bool should_extrapolate_from_neighbour(int neighbourX, int neighbourY, int width, int height, SpaintVoxel::Label label,
                                              const Vector3f& loc, const Vector3f& normal, const Vector3u& colour,
                                              const Vector4f *raycastResult, const Vector3f *surfaceNormals,
                                              const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                              float maxAngleBetweenNormals, float maxSquaredDistanceBetweenColours,
                                              float maxSquaredDistanceBetweenVoxels)
{
  // If the neighbour is outside the raycast result, early out.
  if(neighbourX < 0 || neighbourX >= width || neighbourY < 0 || neighbourY >= height) return false;

  // Look up the position, normal and colour of the neighbouring voxel.
  int neighbourVoxelIndex = neighbourY * width + neighbourX;
  Vector3f neighbourLoc = raycastResult[neighbourVoxelIndex].toVector3();

  bool foundPoint;
  const SpaintVoxel neighbourVoxel = readVoxel(voxelData, indexData, neighbourLoc.toIntRound(), foundPoint);
  if(!foundPoint) return false;

  Vector3f neighbourNormal = surfaceNormals[neighbourVoxelIndex];
  Vector3u neighbourColour = VoxelColourReader<SpaintVoxel::hasColorInformation>::read(neighbourVoxel);

  // Compute the angle between the neighbour's normal and the normal of the voxel of interest.
  float angleBetweenNormals = acosf(dot(normal, neighbourNormal) / (length(normal) * length(neighbourNormal)));

  // Compute the distance between the neighbour's colour and the colour of the voxel of interest.
  Vector3f colourOffset = convert_rgb_to_lab(neighbourColour.toFloat()) - convert_rgb_to_lab(colour.toFloat());
  float squaredDistanceBetweenColours = dot(colourOffset, colourOffset);

  // Compute the squared distance between the neighbour's position and the position of the voxel of interest.
  Vector3f posOffset = neighbourLoc - loc;
  float squaredDistanceBetweenVoxels = dot(posOffset, posOffset);
  float distanceBetweenVoxels = sqrt(squaredDistanceBetweenVoxels);

  // Decide whether or not extrapolation should occur.
  return neighbourVoxel.packedLabel.label == label &&
         angleBetweenNormals <= maxAngleBetweenNormals * distanceBetweenVoxels &&
         squaredDistanceBetweenColours <= maxSquaredDistanceBetweenColours * distanceBetweenVoxels &&
         squaredDistanceBetweenVoxels <= maxSquaredDistanceBetweenVoxels;
}

/**
 * \brief Extrapolates the specified label to the specified voxel as necessary, based on its own properties and those of its neighbours.
 *
 * \param voxelIndex                        The index of the voxel in the raycast result.
 * \param width                             The width of the raycast result.
 * \param height                            The height of the raycast result.
 * \param label                             The label being extrapolated.
 * \param raycastResult                     The raycast result.
 * \param surfaceNormals                    The surface normals for the voxels in the raycast result.
 * \param voxelData                         The scene's voxel data.
 * \param indexData                         The scene's index data.
 * \param maxAngleBetweenNormals            The largest angle allowed between the normals of the neighbour and the voxel of interest if extrapolation is to occur.
 * \param maxSquaredDistanceBetweenColours  The maximum squared distance allowed between the colours of the neighbour and the voxel of interest if extrapolation is to occur.
 * \param maxSquaredDistanceBetweenVoxels   The maximum squared distance allowed between the positions of the neighbour and the voxel of interest if extrapolation is to occur.
 */
_CPU_AND_GPU_CODE_
inline void extrapolate_from_neighbours(int voxelIndex, int width, int height, SpaintVoxel::Label label,
                                        const Vector4f *raycastResult, const Vector3f *surfaceNormals,
                                        SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                        float maxAngleBetweenNormals, float maxSquaredDistanceBetweenColours,
                                        float maxSquaredDistanceBetweenVoxels)
{
  // Look up the position, normal and colour of the specified voxel.
  Vector3f loc = raycastResult[voxelIndex].toVector3();
  Vector3f normal = surfaceNormals[voxelIndex];

  bool foundPoint;
  const SpaintVoxel voxel = readVoxel(voxelData, indexData, loc.toIntRound(), foundPoint);
  if(!foundPoint) return;

  Vector3u colour = VoxelColourReader<SpaintVoxel::hasColorInformation>::read(voxel);

  // Based on these properties and the properties of the neighbouring voxels, decide whether or not
  // the specified voxel should be marked with the label being extrapolated, and mark it if so.
  int x = voxelIndex % width;
  int y = voxelIndex / width;

#define SEFN(nx,ny) should_extrapolate_from_neighbour( \
  nx, ny, width, height, label, loc, normal, colour, \
  raycastResult, surfaceNormals, voxelData, indexData, \
  maxAngleBetweenNormals, maxSquaredDistanceBetweenColours, \
  maxSquaredDistanceBetweenVoxels)

  if((SEFN(x - 2, y) && SEFN(x - 5, y)) ||
     (SEFN(x + 2, y) && SEFN(x + 5, y)) ||
     (SEFN(x, y - 2) && SEFN(x, y - 5)) ||
     (SEFN(x, y + 2) && SEFN(x, y + 5)))
  {
    mark_voxel(loc.toShortRound(), SpaintVoxel::PackedLabel(label, SpaintVoxel::LG_PROPAGATED), NULL, voxelData, indexData);
  }

#undef SPFN
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void interpolate_from_neighbours(int voxelIndex, int width, int height, int maxLabelCount, const Vector4f *raycastResult,
                                       SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                       float maxSquaredDistanceBetweenVoxels)
{
  unsigned char labelCounts[32] = {0,};

  Vector3f loc = raycastResult[voxelIndex].toVector3();
  int x = voxelIndex % width;
  int y = voxelIndex / width;

  bool foundPoint;
  for(int dx = -1; dx <= 1; ++dx)
  {
    for(int dy = -1; dy <= 1; ++dy)
    {
      if(dx == 0 && dy == 0) continue;

      int neighbourX = x + dx, neighbourY = y + dy;
      if(neighbourX < 0 || neighbourX >= width || neighbourY < 0 || neighbourY >= height) continue;

      int neighbourVoxelIndex = neighbourY * width + neighbourX;
      Vector3f neighbourLoc = raycastResult[neighbourVoxelIndex].toVector3();
      const SpaintVoxel neighbourVoxel = readVoxel(voxelData, indexData, neighbourLoc.toIntRound(), foundPoint);
      if(!foundPoint) continue;

      Vector3f posOffset = neighbourLoc - loc;
      float squaredDistanceBetweenVoxels = dot(posOffset, posOffset);

      if(squaredDistanceBetweenVoxels <= maxSquaredDistanceBetweenVoxels)
      {
        ++labelCounts[neighbourVoxel.packedLabel.label];
      }
    }
  }

  SpaintVoxel::Label bestLabel(0);
  int bestLabelCount = 0;
  for(int i = 1; i < maxLabelCount; ++i)
  {
    if(labelCounts[i] > bestLabelCount)
    {
      bestLabel = i;
      bestLabelCount = labelCounts[i];
    }
  }

  if(bestLabel != 0 && bestLabelCount > 5)
  {
    mark_voxel(loc.toShortRound(), SpaintVoxel::PackedLabel(bestLabel, SpaintVoxel::LG_PROPAGATED), NULL, voxelData, indexData);
  }
}

/**
 * \brief Calculates the normal of the specified voxel in the raycast result and writes it into the surface normals array.
 *
 * \param voxelIndex      The index of the voxel whose normal is to be calculated.
 * \param raycastResult   The raycast result.
 * \param voxelData       The scene's voxel data.
 * \param indexData       The scene's index data.
 * \param surfaceNormals  The array into which to write the calculated surface normal.
 */
_CPU_AND_GPU_CODE_
inline void write_surface_normal(int voxelIndex, const Vector4f *raycastResult, const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData, Vector3f *surfaceNormals)
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
