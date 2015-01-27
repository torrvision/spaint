/**
 * spaint: SemanticVisualiser_Shared.h
 */

#ifndef H_SPAINT_SEMANTICVISUALISER_SHARED
#define H_SPAINT_SEMANTICVISUALISER_SHARED

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>
#include <ITMLib/Engine/DeviceAgnostic/ITMVisualisationEngine.h>

#include "../../../util/SpaintVoxel.h"

namespace spaint {

//#################### SHARED HELPER FUNCTIONS ####################

/**
 * \brief Computes the necessary colour for a pixel in a semantic visualiation of the scene.
 *
 * This function is roughly analogous to a pixel shader.
 *
 * \param dest          A location into which to write the computed colour.
 * \param point         The location of the point (if any) on the scene surface that was hit by a ray passing from the camera through the pixel.
 * \param foundPoint    A flag indicating whether or not any point was actually hit by the ray (true if yes; false if no).
 * \param voxelData     The scene's voxel data.
 * \param voxelIndex    The scene's voxel index.
 * \param lightSource   The position of the light source that is illuminating the scene.
 * \param labelColours  The colour map for the semantic labels.
 */
_CPU_AND_GPU_CODE_
inline void shade_pixel_semantic(DEVICEPTR(Vector4u)& dest, const DEVICEPTR(Vector3f)& point, bool foundPoint, const DEVICEPTR(SpaintVoxel) *voxelData,
                                 const DEVICEPTR(ITMVoxelIndex::IndexData) *voxelIndex, Vector3f lightSource, const DEVICEPTR(Vector3u) *labelColours)
{
  dest = Vector4u((uchar)0);
  if(foundPoint)
  {
    // Determine the base colour to use for the pixel based on the semantic label of the voxel we hit.
    // FIXME: Since the labels are currently not set, this currently uses the weight value from the voxel for interest value, but we should change this in due course.
    SpaintVoxel voxel = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)), foundPoint);
    Vector3u colour = labelColours[voxel.w_depth / 32 > 3 ? 3 : voxel.w_depth / 32];

    // Determine the intensity of the pixel using a very simple version of the Lambertian lighting equation.
    Vector3f outNormal;
    float angle;
    computeNormalAndAngle<SpaintVoxel,ITMVoxelIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);
    float intensity = 0.2f + 0.8f * angle;

    // Fill in the final colour for the pixel by scaling the base colour by the intensity.
    dest.x = (uchar)(intensity * colour.r);
    dest.y = (uchar)(intensity * colour.g);
    dest.z = (uchar)(intensity * colour.b);
    dest.w = 255;
  }
}

}

#endif
