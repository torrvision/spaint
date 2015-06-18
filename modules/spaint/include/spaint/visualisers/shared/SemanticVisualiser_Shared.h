/**
 * spaint: SemanticVisualiser_Shared.h
 */

#ifndef H_SPAINT_SEMANTICVISUALISER_SHARED
#define H_SPAINT_SEMANTICVISUALISER_SHARED

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>
#include <ITMLib/Engine/DeviceAgnostic/ITMVisualisationEngine.h>

#include "../../util/SpaintVoxel.h"

namespace spaint {

//#################### SHARED HELPER FUNCTIONS ####################

/**
 * \brief Computes the colour for a pixel in a semantic visualisation of the scene.
 *
 * This function is roughly analogous to a pixel shader.
 *
 * \param dest          A location into which to write the computed colour.
 * \param point         The location of the point (if any) on the scene surface that was hit by a ray passing from the camera through the pixel.
 * \param foundPoint    A flag indicating whether or not any point was actually hit by the ray (true if yes; false if no).
 * \param voxelData     The scene's voxel data.
 * \param voxelIndex    The scene's voxel index.
 * \param labelColours  The colour map for the semantic labels.
 * \param viewerPos     The position of the viewer (in voxel coordinates).
 * \param lightPos      The position of the light source that is illuminating the scene (in voxel coordinates).
 * \param usePhong      Whether or not to use Phong lighting.
 * \param labelAlpha    The proportion (in the range [0,1]) of the final pixel colour that should be based on the voxel's semantic label rather than its scene colour.
 */
_CPU_AND_GPU_CODE_
inline void shade_pixel_semantic(Vector4u& dest, const Vector3f& point, bool foundPoint, const SpaintVoxel *voxelData,
                                 const ITMVoxelIndex::IndexData *voxelIndex, const Vector3u *labelColours,
                                 const Vector3f& viewerPos, const Vector3f& lightPos, bool usePhong, const float labelAlpha)
{
  const float ambient = usePhong ? 0.3f : 0.2f;
  const float lambertianCoefficient = usePhong ? 0.35f : 0.8f;
  const float phongCoefficient = 0.35f;
  const float phongExponent = 20.0f;

  dest = Vector4u((uchar)0);
  if(foundPoint)
  {
    // Determine the base colour to use for the pixel based on the semantic label of the voxel we hit and its scene colour (if available).
    const SpaintVoxel voxel = readVoxel(voxelData, voxelIndex, point.toIntRound(), foundPoint);
    const Vector3u labelColour = labelColours[voxel.label];
    Vector3u colour;
    if(SpaintVoxel::hasColorInformation)
    {
      const Vector3u sceneColour = VoxelColourReader<SpaintVoxel::hasColorInformation>::read(voxel);
      colour = (labelAlpha * labelColour.toFloat() + (1.0f - labelAlpha) * sceneColour.toFloat()).toUChar();
    }
    else colour = labelColour;

    // Calculate the Lambertian lighting term.
    Vector3f L = normalize(lightPos - point);
    Vector3f N = 0.0f;
    float NdotL = 0.0f;
    computeNormalAndAngle<SpaintVoxel,ITMVoxelIndex>(foundPoint, point, voxelData, voxelIndex, L, N, NdotL);
    float lambertian = CLAMP(NdotL, 0.0f, 1.0f);

    // Determine the intensity of the pixel using the Lambertian lighting equation.
    float intensity = ambient + lambertianCoefficient * lambertian;

    // If we're using Phong lighting:
    if(usePhong)
    {
      // Calculate the Phong lighting term.
      Vector3f R = 2.0f * N * NdotL - L;
      Vector3f V = normalize(viewerPos - point);
      float phong = pow(CLAMP(dot(R,V), 0.0f, 1.0f), phongExponent);

      // Add the Phong lighting term to the intensity.
      intensity += phongCoefficient * phong;
    }

    // Fill in the final colour for the pixel by scaling the base colour by the intensity.
    dest.x = (uchar)(intensity * colour.r);
    dest.y = (uchar)(intensity * colour.g);
    dest.z = (uchar)(intensity * colour.b);
    dest.w = 255;
  }
}

}

#endif
