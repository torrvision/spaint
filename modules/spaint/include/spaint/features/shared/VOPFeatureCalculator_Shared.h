/**
 * spaint: VOPFeatureCalculator_Shared.h
 */

#ifndef H_SPAINT_VOPFEATURECALCULATOR_SHARED
#define H_SPAINT_VOPFEATURECALCULATOR_SHARED

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

#include "../../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief Generates a unit vector that is perpendicular to the specified plane normal.
 *
 * The vector generated will be the normalised cross product of the specified plane normal and another vector
 * that is non-parallel to the normal. This non-parallel vector will be the up vector (0,0,1), unless that is
 * parallel to the normal, in which case (1,0,0) will be used instead.
 *
 * \param n The normal of the plane in which we want to generate the unit vector.
 * \return  The unit coplanar vector v as specified, satisfying v.dot(n) == 0.
 */
_CPU_AND_GPU_CODE_
inline Vector3f generate_arbitrary_coplanar_unit_vector(const Vector3f& n)
{
  Vector3f up(0.0f, 0.0f, 1.0f);
  if(fabs(n.x) < 1e-3f && fabs(n.y) < 1e-3f)
  {
    // Special Case: n is too close to the vertical and hence n x up is roughly equal to (0,0,0).
    // Use (1,0,0) instead of up and apply the same method as in the else clause.
    up = Vector3f(1.0f, 0.0f, 0.0f);
    return normalize(cross(n, Vector3f(1.0f, 0.0f, 0.0f)));
  }
  else
  {
    // The normalized cross product of n and up satisfies the requirements of being
    // unit length and perpendicular to n (since we dealt with the special case where
    // n x up is zero, in all other cases it must be non-zero and we can normalize it
    // to give us a unit vector).
    return normalize(cross(n, up));
  }
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void generate_coordinate_system(int voxelLocationIndex, const Vector3f *surfaceNormals, Vector3f *xAxes, Vector3f *yAxes)
{
  Vector3f n = surfaceNormals[voxelLocationIndex];
  Vector3f xAxis = generate_arbitrary_coplanar_unit_vector(n);
  xAxes[voxelLocationIndex] = xAxis;
  yAxes[voxelLocationIndex] = cross(xAxis, n);
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void generate_rgb_patch(int voxelLocationIndex, const Vector3s *voxelLocations, const Vector3f *xAxes, const Vector3f *yAxes,
                               const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData, size_t patchSize, float patchSpacing,
                               size_t featureCount, float *features)
{
  // Check that the voxel exists. If it doesn't, early out.
  Vector3f centre = voxelLocations[voxelLocationIndex].toFloat();
  bool isFound;
  int voxelAddress = findVoxel(indexData, centre.toInt(), isFound);
  if(!isFound) return;

  // Generate an RGB patch around the voxel on a patchSize * patchSize grid aligned with the voxel's x and y axes.
  int halfPatchSize = static_cast<int>(patchSize - 1) / 2;
  Vector3f xAxis = xAxes[voxelLocationIndex] * patchSpacing;
  Vector3f yAxis = yAxes[voxelLocationIndex] * patchSpacing;

  // For each pixel in the patch:
  size_t offset = voxelLocationIndex * featureCount;
  for(int y = -halfPatchSize; y <= halfPatchSize; ++y)
  {
    Vector3f yLoc = centre + static_cast<float>(y) * yAxis;
    for(int x = -halfPatchSize; x <= halfPatchSize; ++x)
    {
      // Compute the location of the pixel in world space.
      Vector3i loc = (yLoc + static_cast<float>(x) * xAxis).toIntRound();

      // If there is a voxel at that location, get its colour; otherwise, default to magenta.
      Vector3u clr(255, 0, 255);
      SpaintVoxel voxel = readVoxel(voxelData, indexData, loc, isFound);
      if(isFound) clr = voxel.clr;

      // Write the colour values into the relevant places in the features array.
      features[offset++] = clr.r;
      features[offset++] = clr.g;
      features[offset++] = clr.b;
    }
  }
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void write_surface_normal(int voxelLocationIndex, const Vector3s *voxelLocations, const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                 Vector3f *surfaceNormals)
{
  surfaceNormals[voxelLocationIndex] = computeSingleNormalFromSDF(voxelData, indexData, voxelLocations[voxelLocationIndex].toFloat());
}

}

#endif
